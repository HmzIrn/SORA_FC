#include "usb_command.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "usb_telemetry.h"

#define RX_RING_SIZE     (256U)              /* power of two */
#define RX_RING_MASK     (RX_RING_SIZE - 1U)
#define LINE_MAX_LEN     (96U)
#define OUT_Q_LEN        (8U)
#define OUT_LINE_MAX     (160U)
#define MAX_PARAMS       (48U)
#define MAX_TOKENS       (4U)

/* ------------------------------------------------------------------------ */
/* State                                                                    */
/* ------------------------------------------------------------------------ */

static USB_Command_Bindings_t g_bind;
static USB_Param_t  g_params[MAX_PARAMS];
static uint32_t     g_param_count = 0U;
static bool         g_dirty = false;

/* RX ring: producer = USB interrupt, consumer = main loop. */
static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0U;
static volatile uint16_t rx_tail = 0U;

/* Line assembly (main loop only). */
static char     line_buf[LINE_MAX_LEN];
static uint32_t line_len = 0U;
static bool     line_too_long = false;
static bool     line_ready = false;

/* Output queue (main loop only). */
static char     out_q[OUT_Q_LEN][OUT_LINE_MAX];
static uint8_t  out_head = 0U;
static uint8_t  out_count = 0U;
static uint32_t out_dropped = 0U;

/* Multi-line replies are produced one line at a time. */
typedef enum { GEN_NONE = 0, GEN_HELP, GEN_LIST } Gen_t;
static Gen_t    gen = GEN_NONE;
static uint32_t gen_index = 0U;
static uint32_t gen_matched = 0U;
static char     gen_prefix[24];

static bool     pending_reboot = false;

static const char *const help_lines[] =
{
    "HELP,HELP | STATUS | LIST [prefix] | GET <name> | SET <name> <value>",
    "HELP,SAVE | LOAD | CLEAR | REBOOT | CAL FULL | CAL BIAS",
    "HELP,groups: cal.* (gyro/accel calibration)  mekf.* (filter, residual bias)  mix.* (PID, limits, trims)",
    "HELP,SET changes one value live; SAVE stores everything to flash"
};

#define HELP_COUNT  ((uint32_t)(sizeof(help_lines) / sizeof(help_lines[0])))

/* ------------------------------------------------------------------------ */
/* Small helpers                                                            */
/* ------------------------------------------------------------------------ */

/* Float -> text without floating-point printf (newlib-nano friendly). */
static void fmt_float(char *dst, size_t size, float v)
{
    if (isnan(v))
    {
        snprintf(dst, size, "nan");
        return;
    }

    if (isinf(v))
    {
        snprintf(dst, size, "%s", (v < 0.0f) ? "-inf" : "inf");
        return;
    }

    const bool neg = (v < 0.0f);
    const float a = neg ? -v : v;

    if (a >= 1.0e9f)
    {
        snprintf(dst, size, "%s1e9+", neg ? "-" : "");
        return;
    }

    /* More decimals for small values so tiny noise parameters stay visible. */
    uint32_t decimals;
    float scale;

    if (a >= 1.0f)        { decimals = 6U; scale = 1.0e6f; }
    else if (a >= 0.001f) { decimals = 7U; scale = 1.0e7f; }
    else                  { decimals = 9U; scale = 1.0e9f; }

    uint32_t ip = (uint32_t)a;
    uint32_t fp = (uint32_t)((a - (float)ip) * scale + 0.5f);
    const uint32_t limit = (uint32_t)scale;

    if (fp >= limit)
    {
        ip++;
        fp -= limit;
    }

    char frac[12];
    snprintf(frac, sizeof(frac), "%0*lu", (int)decimals, (unsigned long)fp);

    /* Trim trailing zeros, keep at least one digit. */
    size_t n = strlen(frac);
    while (n > 1U && frac[n - 1U] == '0')
        frac[--n] = '\0';

    const bool is_zero = (ip == 0U) && (fp == 0U);

    snprintf(dst, size, "%s%lu.%s",
             (neg && !is_zero) ? "-" : "",
             (unsigned long)ip, frac);
}

static bool parse_float(const char *s, float *out)
{
    char *end = NULL;
    const float v = strtof(s, &end);

    if (end == s || *end != '\0' || !isfinite(v))
        return false;

    *out = v;
    return true;
}

static bool parse_bool(const char *s, bool *out)
{
    if (strcmp(s, "1") == 0 || strcmp(s, "on") == 0 || strcmp(s, "true") == 0)
    {
        *out = true;
        return true;
    }

    if (strcmp(s, "0") == 0 || strcmp(s, "off") == 0 || strcmp(s, "false") == 0)
    {
        *out = false;
        return true;
    }

    return false;
}

static float param_get(const USB_Param_t *p)
{
    if (p->type == USB_PARAM_BOOL)
        return (*(const bool *)p->ptr) ? 1.0f : 0.0f;

    return *(const float *)p->ptr;
}

static void fmt_param(char *dst, size_t size, const USB_Param_t *p)
{
    if (p->type == USB_PARAM_BOOL)
        snprintf(dst, size, "%u", (*(const bool *)p->ptr) ? 1U : 0U);
    else
        fmt_float(dst, size, *(const float *)p->ptr);
}

static const USB_Param_t *param_find(const char *name)
{
    for (uint32_t i = 0U; i < g_param_count; i++)
    {
        if (strcmp(g_params[i].name, name) == 0)
            return &g_params[i];
    }

    return NULL;
}

/* ------------------------------------------------------------------------ */
/* Output queue                                                             */
/* ------------------------------------------------------------------------ */

static bool push_line(const char *text)
{
    if (out_count >= OUT_Q_LEN)
    {
        out_dropped++;
        return false;
    }

    char *slot = out_q[(out_head + out_count) % OUT_Q_LEN];
    size_t n = strlen(text);

    if (n > (OUT_LINE_MAX - 3U))
        n = OUT_LINE_MAX - 3U;

    memcpy(slot, text, n);
    slot[n++] = '\r';
    slot[n++] = '\n';
    slot[n] = '\0';

    out_count++;
    return true;
}

static void flush_one(void)
{
    if (out_count == 0U)
        return;

    const char *slot = out_q[out_head];

    if (USB_Telemetry_SendText(slot, (uint16_t)strlen(slot)))
    {
        out_head = (uint8_t)((out_head + 1U) % OUT_Q_LEN);
        out_count--;
    }
}

void USB_Command_Notify(const char *line)
{
    if (line != NULL)
        (void)push_line(line);
}

/* ------------------------------------------------------------------------ */
/* Parameter table                                                          */
/* ------------------------------------------------------------------------ */

static void add_param(const char *name, void *ptr, USB_ParamType_t type,
                      float min, float max, USB_ParamApply_t apply)
{
    if (g_param_count >= MAX_PARAMS)
        return;

    USB_Param_t *p = &g_params[g_param_count++];

    p->name = name;
    p->ptr = ptr;
    p->type = type;
    p->min = min;
    p->max = max;
    p->apply = apply;
}

#define ADD_F(name, ptr, mn, mx) \
    add_param((name), (ptr), USB_PARAM_FLOAT, (mn), (mx), USB_PARAM_APPLY_NONE)

static void build_param_table(void)
{
    IMU_CalibrationResult_t *c = g_bind.imu_result;
    MEKF_Config_t *k = g_bind.mekf_cfg;
    MotorMixer_Config_t *m = g_bind.mixer_cfg;
    float *b = g_bind.mekf_bias_dps;

    g_param_count = 0U;

    /* --- IMU calibration (gyro bias [dps], accel offset [g], accel scale) --- */
    ADD_F("cal.gyro_bias_x",   &c->gyro_bias_dps[0],  -50.0f, 50.0f);
    ADD_F("cal.gyro_bias_y",   &c->gyro_bias_dps[1],  -50.0f, 50.0f);
    ADD_F("cal.gyro_bias_z",   &c->gyro_bias_dps[2],  -50.0f, 50.0f);
    ADD_F("cal.accel_offset_x", &c->accel_offset_g[0], -1.0f, 1.0f);
    ADD_F("cal.accel_offset_y", &c->accel_offset_g[1], -1.0f, 1.0f);
    ADD_F("cal.accel_offset_z", &c->accel_offset_g[2], -1.0f, 1.0f);
    ADD_F("cal.accel_scale_x",  &c->accel_scale[0],     0.5f, 2.0f);
    ADD_F("cal.accel_scale_y",  &c->accel_scale[1],     0.5f, 2.0f);
    ADD_F("cal.accel_scale_z",  &c->accel_scale[2],     0.5f, 2.0f);

    /* --- MEKF: residual gyro bias [dps] and tuning --- */
    add_param("mekf.bias_x", &b[0], USB_PARAM_FLOAT, -50.0f, 50.0f, USB_PARAM_APPLY_MEKF_BIAS);
    add_param("mekf.bias_y", &b[1], USB_PARAM_FLOAT, -50.0f, 50.0f, USB_PARAM_APPLY_MEKF_BIAS);
    add_param("mekf.bias_z", &b[2], USB_PARAM_FLOAT, -50.0f, 50.0f, USB_PARAM_APPLY_MEKF_BIAS);
    ADD_F("mekf.gyro_noise",         &k->gyro_noise,         1.0e-5f, 1.0f);
    ADD_F("mekf.gyro_bias_rw",       &k->gyro_bias_rw,       1.0e-8f, 0.1f);
    ADD_F("mekf.accel_noise",        &k->accel_noise,        1.0e-3f, 1.0f);
    ADD_F("mekf.init_att_std",       &k->init_att_std,       1.0e-3f, 1.5f);
    ADD_F("mekf.init_bias_std",      &k->init_bias_std,      1.0e-4f, 1.0f);
    ADD_F("mekf.accel_soft_tol",     &k->accel_soft_tol,     1.0e-3f, 1.0f);
    ADD_F("mekf.accel_reject_tol",   &k->accel_reject_tol,   1.0e-2f, 2.0f);
    ADD_F("mekf.calib_duration_s",   &k->calib_duration_s,   0.2f,   60.0f);
    ADD_F("mekf.calib_max_gyro_std", &k->calib_max_gyro_std, 1.0e-3f, 5.0f);

    /* --- Motor mixer: PID, limits, trims --- */
    ADD_F("mix.kp_rp",       &m->kp_rp,        0.0f, 10.0f);
    ADD_F("mix.ki_rp",       &m->ki_rp,        0.0f, 10.0f);
    ADD_F("mix.kd_rp",       &m->kd_rp,        0.0f, 10.0f);
    ADD_F("mix.kp_yaw",      &m->kp_yaw,       0.0f, 10.0f);
    ADD_F("mix.ki_yaw",      &m->ki_yaw,       0.0f, 10.0f);
    ADD_F("mix.kd_yaw",      &m->kd_yaw,       0.0f, 10.0f);
    ADD_F("mix.i_limit",     &m->i_limit,      0.0f, 1.0f);
    add_param("mix.use_integral", &m->use_integral, USB_PARAM_BOOL, 0.0f, 1.0f,
              USB_PARAM_APPLY_NONE);
    ADD_F("mix.idle_thrust", &m->idle_thrust,  0.0f, 0.5f);
    ADD_F("mix.twr",         &m->twr,          1.0f, 10.0f);
    ADD_F("mix.max_tilt_deg", &m->max_tilt_deg, 5.0f, 85.0f);
    ADD_F("mix.min_cos_tilt", &m->min_cos_tilt, 0.05f, 1.0f);
    ADD_F("mix.max_rpm",     &m->max_rpm,      100.0f, 100000.0f);
    ADD_F("mix.trim_roll",   &m->trim[0],      -0.2f, 0.2f);
    ADD_F("mix.trim_pitch",  &m->trim[1],      -0.2f, 0.2f);
    ADD_F("mix.trim_yaw",    &m->trim[2],      -0.2f, 0.2f);
}

uint32_t USB_Command_ParamCount(void)
{
    return g_param_count;
}

/* ------------------------------------------------------------------------ */
/* Command execution                                                        */
/* ------------------------------------------------------------------------ */

static void reply(const char *text)
{
    (void)push_line(text);
}

static void reply_err(const char *reason)
{
    char buf[OUT_LINE_MAX];

    snprintf(buf, sizeof(buf), "ERR,%s", reason);
    reply(buf);
}

static uint32_t tokenize(char *s, char *tok[], uint32_t max)
{
    uint32_t n = 0U;
    bool in_tok = false;

    for (; *s != '\0'; s++)
    {
        const char c = *s;
        const bool sep = (c == ' ' || c == ',' || c == '=' || c == '\t');

        if (sep)
        {
            *s = '\0';
            in_tok = false;
        }
        else if (!in_tok)
        {
            if (n >= max)
                return n + 1U;   /* too many tokens */

            tok[n++] = s;
            in_tok = true;
        }
    }

    return n;
}

static bool busy(void)
{
    return (g_bind.hooks.busy != NULL) && g_bind.hooks.busy();
}

static void cmd_get(const char *name)
{
    const USB_Param_t *p = param_find(name);
    char val[24];
    char buf[OUT_LINE_MAX];

    if (p == NULL)
    {
        snprintf(buf, sizeof(buf), "UNKNOWN_PARAM,%s", name);
        reply_err(buf);
        return;
    }

    fmt_param(val, sizeof(val), p);
    snprintf(buf, sizeof(buf), "OK,GET,%s,%s", p->name, val);
    reply(buf);
}

static void cmd_set(const char *name, const char *value)
{
    const USB_Param_t *p = param_find(name);
    char buf[OUT_LINE_MAX];
    char a[24];
    char b[24];

    if (p == NULL)
    {
        snprintf(buf, sizeof(buf), "UNKNOWN_PARAM,%s", name);
        reply_err(buf);
        return;
    }

    if (busy())
    {
        reply_err("BUSY,calibration in progress");
        return;
    }

    if (p->type == USB_PARAM_BOOL)
    {
        bool bv;

        if (!parse_bool(value, &bv))
        {
            snprintf(buf, sizeof(buf), "BAD_VALUE,%s,expected 0/1/on/off", p->name);
            reply_err(buf);
            return;
        }

        *(bool *)p->ptr = bv;
    }
    else
    {
        float v;

        if (!parse_float(value, &v))
        {
            snprintf(buf, sizeof(buf), "BAD_VALUE,%s", p->name);
            reply_err(buf);
            return;
        }

        if (v < p->min || v > p->max)
        {
            fmt_float(a, sizeof(a), p->min);
            fmt_float(b, sizeof(b), p->max);
            snprintf(buf, sizeof(buf), "RANGE,%s,%s,%s", p->name, a, b);
            reply_err(buf);
            return;
        }

        *(float *)p->ptr = v;
    }

    if (g_bind.hooks.param_changed != NULL)
        g_bind.hooks.param_changed(p);

    g_dirty = true;

    fmt_param(a, sizeof(a), p);
    snprintf(buf, sizeof(buf), "OK,SET,%s,%s,UNSAVED", p->name, a);
    reply(buf);
}

static void execute(char *line)
{
    char *tok[MAX_TOKENS];
    const uint32_t n = tokenize(line, tok, MAX_TOKENS);

    if (n == 0U)
        return;

    if (n > MAX_TOKENS)
    {
        reply_err("TOO_MANY_ARGS");
        return;
    }

    const char *cmd = tok[0];

    if (strcmp(cmd, "help") == 0)
    {
        gen = GEN_HELP;
        gen_index = 0U;
    }
    else if (strcmp(cmd, "status") == 0)
    {
        char buf[OUT_LINE_MAX];

        if (g_bind.hooks.format_status != NULL &&
            g_bind.hooks.format_status(buf, sizeof(buf)) > 0)
        {
            reply(buf);
        }
        else
        {
            reply_err("UNSUPPORTED");
        }
    }
    else if (strcmp(cmd, "list") == 0)
    {
        gen_prefix[0] = '\0';

        if (n >= 2U)
        {
            strncpy(gen_prefix, tok[1], sizeof(gen_prefix) - 1U);
            gen_prefix[sizeof(gen_prefix) - 1U] = '\0';
        }

        gen = GEN_LIST;
        gen_index = 0U;
        gen_matched = 0U;
    }
    else if (strcmp(cmd, "get") == 0)
    {
        if (n != 2U)
            reply_err("USAGE,GET <name>");
        else
            cmd_get(tok[1]);
    }
    else if (strcmp(cmd, "set") == 0)
    {
        if (n != 3U)
            reply_err("USAGE,SET <name> <value>");
        else
            cmd_set(tok[1], tok[2]);
    }
    else if (strcmp(cmd, "save") == 0)
    {
        if (g_bind.hooks.save == NULL)
            reply_err("UNSUPPORTED");
        else if (busy())
            reply_err("BUSY,calibration in progress");
        else if (g_bind.hooks.save())
        {
            g_dirty = false;
            reply("OK,SAVE");
        }
        else
            reply_err("SAVE_FAILED");
    }
    else if (strcmp(cmd, "load") == 0)
    {
        if (g_bind.hooks.load == NULL)
            reply_err("UNSUPPORTED");
        else if (busy())
            reply_err("BUSY,calibration in progress");
        else if (g_bind.hooks.load())
        {
            g_dirty = false;
            reply("OK,LOAD");
        }
        else
            reply_err("NO_STORED_CONFIG");
    }
    else if (strcmp(cmd, "clear") == 0)
    {
        if (g_bind.hooks.clear == NULL)
            reply_err("UNSUPPORTED");
        else if (g_bind.hooks.clear())
            reply("OK,CLEAR,stored config erased; next boot recalibrates");
        else
            reply_err("CLEAR_FAILED");
    }
    else if (strcmp(cmd, "cal") == 0)
    {
        if (n != 2U)
            reply_err("USAGE,CAL FULL|BIAS");
        else if (strcmp(tok[1], "full") == 0)
        {
            if (g_bind.hooks.start_full_calibration != NULL &&
                g_bind.hooks.start_full_calibration())
                reply("OK,CAL,FULL,keep the board still; follow the face prompts");
            else
                reply_err("CAL_FULL_REFUSED");
        }
        else if (strcmp(tok[1], "bias") == 0)
        {
            if (g_bind.hooks.start_bias_calibration != NULL &&
                g_bind.hooks.start_bias_calibration())
                reply("OK,CAL,BIAS,keep the board still");
            else
                reply_err("CAL_BIAS_REFUSED,calibrate first or wait for the filter");
        }
        else
            reply_err("USAGE,CAL FULL|BIAS");
    }
    else if (strcmp(cmd, "reboot") == 0)
    {
        reply("OK,REBOOT");
        pending_reboot = true;
    }
    else
    {
        char buf[OUT_LINE_MAX];

        snprintf(buf, sizeof(buf), "UNKNOWN_CMD,%s", cmd);
        reply_err(buf);
    }
}

/* ------------------------------------------------------------------------ */
/* Multi-line replies                                                       */
/* ------------------------------------------------------------------------ */

static bool prefix_match(const char *name, const char *prefix)
{
    return strncmp(name, prefix, strlen(prefix)) == 0;
}

static void gen_step(void)
{
    char buf[OUT_LINE_MAX];

    if (gen == GEN_HELP)
    {
        if (gen_index < HELP_COUNT)
        {
            (void)push_line(help_lines[gen_index++]);
        }
        else
        {
            (void)push_line("OK,HELP");
            gen = GEN_NONE;
        }

        return;
    }

    /* GEN_LIST */
    while (gen_index < g_param_count &&
           !prefix_match(g_params[gen_index].name, gen_prefix))
    {
        gen_index++;
    }

    if (gen_index < g_param_count)
    {
        const USB_Param_t *p = &g_params[gen_index++];
        char v[24];
        char lo[24];
        char hi[24];

        fmt_param(v, sizeof(v), p);
        fmt_float(lo, sizeof(lo), p->min);
        fmt_float(hi, sizeof(hi), p->max);

        snprintf(buf, sizeof(buf), "PARAM,%s,%s,%s,%s%s", p->name, v, lo, hi,
                 (p->type == USB_PARAM_BOOL) ? ",bool" : "");
        (void)push_line(buf);
        gen_matched++;
    }
    else
    {
        snprintf(buf, sizeof(buf), "OK,LIST,%lu", (unsigned long)gen_matched);
        (void)push_line(buf);
        gen = GEN_NONE;
    }
}

/* ------------------------------------------------------------------------ */
/* Public API                                                               */
/* ------------------------------------------------------------------------ */

void USB_Command_Init(const USB_Command_Bindings_t *bindings)
{
    if (bindings == NULL)
        return;

    g_bind = *bindings;

    rx_head = 0U;
    rx_tail = 0U;
    line_len = 0U;
    line_too_long = false;
    line_ready = false;
    out_head = 0U;
    out_count = 0U;
    out_dropped = 0U;
    gen = GEN_NONE;
    pending_reboot = false;
    g_dirty = false;

    if (g_bind.imu_result != NULL && g_bind.mekf_cfg != NULL &&
        g_bind.mekf_bias_dps != NULL && g_bind.mixer_cfg != NULL)
    {
        build_param_table();
    }
    else
    {
        g_param_count = 0U;
    }
}

void USB_Command_Feed(const uint8_t *data, uint32_t len)
{
    if (data == NULL)
        return;

    for (uint32_t i = 0U; i < len; i++)
    {
        const uint16_t next = (uint16_t)((rx_head + 1U) & RX_RING_MASK);

        if (next == rx_tail)
            return;   /* ring full: drop the rest (host should pace commands) */

        rx_ring[rx_head] = data[i];
        rx_head = next;
    }
}

bool USB_Command_IsDirty(void)
{
    return g_dirty;
}

void USB_Command_ClearDirty(void)
{
    g_dirty = false;
}

/* Pull bytes from the ring until one complete line is assembled. */
static void assemble_line(void)
{
    while (!line_ready && rx_tail != rx_head)
    {
        const uint8_t c = rx_ring[rx_tail];

        rx_tail = (uint16_t)((rx_tail + 1U) & RX_RING_MASK);

        if (c == '\r' || c == '\n')
        {
            if (line_len > 0U || line_too_long)
                line_ready = true;

            continue;
        }

        if (line_len < (LINE_MAX_LEN - 1U))
        {
            line_buf[line_len++] = (c >= 'A' && c <= 'Z') ? (char)(c + 32U) : (char)c;
        }
        else
        {
            line_too_long = true;
        }
    }
}

void USB_Command_Process(void)
{
    flush_one();

    if (gen != GEN_NONE)
    {
        if (out_count < OUT_Q_LEN)
            gen_step();

        return;
    }

    /* Replies stay in order: wait until everything queued has been sent. */
    if (out_count != 0U)
        return;

    if (pending_reboot)
    {
        pending_reboot = false;

        if (g_bind.hooks.reboot != NULL)
            g_bind.hooks.reboot();

        return;
    }

    assemble_line();

    if (!line_ready)
        return;

    if (line_too_long)
    {
        reply_err("LINE_TOO_LONG");
    }
    else
    {
        line_buf[line_len] = '\0';
        execute(line_buf);
    }

    line_len = 0U;
    line_too_long = false;
    line_ready = false;
}

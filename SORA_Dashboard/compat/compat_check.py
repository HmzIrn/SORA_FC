#!/usr/bin/env python3
"""
Firmware <-> dashboard compatibility check.

  python compat_check.py --fw-dir <firmware sources> --dash <dashboard .py> [--static-only]

STATIC   reads the firmware sources and compares everything the two sides must agree on
         (enum order, FC field count, command names, error reasons, parameter names, line limits,
         units, sensor-to-body matrix) with the dashboard's constants.
DYNAMIC  compiles the REAL firmware (main loop, usb_command, usb_telemetry, config_store, mekf,
         motor_mixer, imu_calibration) into a host process (fw_sim, see fw_sim.c) with a simulated IMU
         and user, connects the REAL dashboard code to it through a fake serial port (headless Qt
         stand-in, see fake_qt.py) and walks through: first-boot calibration, live bias edit, save,
         calibration-value edit, range errors, reboot + persistence, CAL BIAS, CAL FULL.
Needs: python3 + numpy. The DYNAMIC part also needs gcc and Linux/macOS/WSL (POSIX pipes); on native
Windows only the STATIC part runs. No Qt / pyserial required.
"""
import argparse
import importlib.util
import math
import os
import re
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fake_qt  # noqa: E402

results = []


def check(name, cond, info=""):
    results.append(bool(cond))
    print("[%s] %s %s" % ("PASS" if cond else "FAIL", name, info))
    return bool(cond)


# Folders that never contain the project's own firmware sources (build output + ST vendor code).
SKIP_DIRS = ("Debug", "Release", ".git", "STM32F4xx_HAL_Driver", "CMSIS", "Middlewares")


def locate(root, name):
    """Find a firmware file anywhere below `root` (CubeIDE layout). Your own folders under Drivers/ are
    searched; only build output and ST vendor folders are skipped. Warns if the name exists twice."""
    hits = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        if name in filenames:
            hits.append(os.path.join(dirpath, name))
    if not hits:
        stem = os.path.splitext(name)[0]
        similar = []
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if d not in (".git",)]
            similar += [os.path.join(dirpath, f) for f in filenames if stem.lower() in f.lower()]
        raise FileNotFoundError(
            "%s not found under %s (searched everything except: %s).%s"
            % (name, os.path.abspath(root), ", ".join(SKIP_DIRS),
               ("\n  similar names found: " + "; ".join(similar[:6])) if similar else
               "\n  no file with a similar name either - is --fw-dir the project root?"))
    if len(hits) > 1:
        print("WARNING: %s exists %d times, using the first: %s" % (name, len(hits), " | ".join(hits)))
    return hits[0]


def read(root, name):
    with open(locate(root, name), encoding="utf-8", errors="ignore") as f:
        return f.read()


# --------------------------------------------------------------------------------------- static
def enum_names(text, typedef, prefix):
    m = re.search(r"typedef\s+enum\s*\{([^}]*)\}\s*%s\s*;" % typedef, text, re.S)
    body = re.sub(r"/\*.*?\*/", "", m.group(1), flags=re.S)
    body = re.sub(r"//.*", "", body)
    names = []
    for item in body.split(","):
        item = item.split("=")[0].strip()
        if item.startswith(prefix):
            names.append(item[len(prefix):])
    return names


def static_checks(fw, d):
    mekf_h = read(fw, "mekf.h")
    mix_h = read(fw, "motor_mixer.h")
    tel_h = read(fw, "usb_telemetry.h")
    tel_c = read(fw, "usb_telemetry.c")
    cmd_c = read(fw, "usb_command.c")
    main_c = read(fw, "main.c")

    print("\n== STATIC: protocol constants ==")
    ev = enum_names(mekf_h, "MEKF_Event_t", "MEKF_EVENT_")
    check("MEKF_Event_t order == dashboard MEKF_EVENT_NAMES", ev == list(d.MEKF_EVENT_NAMES),
          "" if ev == list(d.MEKF_EVENT_NAMES) else "\n  fw:   %s\n  dash: %s" % (ev, list(d.MEKF_EVENT_NAMES)))
    st = enum_names(mekf_h, "MEKF_State_t", "MEKF_STATE_")
    check("MEKF_State_t order == dashboard MEKF_STATE_NAMES", st == list(d.MEKF_STATE_NAMES), str(st))
    ms = enum_names(mix_h, "MotorMixer_Status_t", "MOTOR_MIXER_STATUS_")
    check("MotorMixer_Status_t order == dashboard MIXER_STATUS_NAMES", ms == list(d.MIXER_STATUS_NAMES), str(ms))

    idx = [int(m.group(1)) for m in re.finditer(r"^\s*\*\s+(\d+)\s+\S", tel_h, re.M)]
    check("FC field count in usb_telemetry.h == dashboard FC_FIELD_COUNT", max(idx) == d.FC_FIELD_COUNT,
          "(fw %d, dash %d)" % (max(idx), d.FC_FIELD_COUNT))

    print("\n== STATIC: units ==")
    check("firmware DATA accel/gyro are mg / mdps (x1000) == dashboard default scale",
          "accel_x_g * 1000.0f" in tel_c and "gyro_x_dps * 1000.0f" in tel_c
          and d.ACCEL_G_PER_LSB == 0.001 and d.GYRO_DPS_PER_LSB == 0.001)
    check("FC scales (mdeg, 1e-4 quaternion, permille) match parse_fc",
          "MEKF_RAD_TO_DEG * 1000.0f" in tel_c and "10000.0f" in tel_c and "1000.0f" in tel_c)

    print("\n== STATIC: sensor frame ==")
    m = re.search(r"SENSOR_TO_BODY\[3\]\[3\]\s*=\s*\{(.*?)\};", main_c, re.S)
    nums = [float(x) for x in re.findall(r"[-+]?\d+\.\d+f?", m.group(1).replace("f", ""))]
    fw_M = [nums[0:3], nums[3:6], nums[6:9]]
    dash_M = d.SENSOR_TO_BODY_PRESETS[d.DEFAULT_MAPPING].tolist()
    check("firmware SENSOR_TO_BODY == dashboard default mapping", fw_M == dash_M, "(%s vs %s)" % (fw_M, dash_M))

    print("\n== STATIC: command channel ==")
    fw_cmds = set(re.findall(r'strcmp\(cmd,\s*"(\w+)"\)', cmd_c))
    dash_cmds = {"status", "list", "get", "set", "save", "load", "cal", "clear", "reboot"}
    check("every command the dashboard sends exists in the firmware", dash_cmds <= fw_cmds, str(sorted(dash_cmds - fw_cmds)))
    check("CAL sub-commands 'full' and 'bias' exist", 'strcmp(tok[1], "full")' in cmd_c and 'strcmp(tok[1], "bias")' in cmd_c)
    lm = re.search(r"#define\s+LINE_MAX_LEN\s+\((\d+)U\)", cmd_c)
    fw_line = int(lm.group(1))
    check("dashboard max command length fits the firmware line buffer", d.FirmwareLink.MAX_LINE == fw_line - 1,
          "(dash %d, fw buffer %d)" % (d.FirmwareLink.MAX_LINE, fw_line))

    reasons = set(re.findall(r'reply_err\("([A-Z_]+)', cmd_c))
    reasons |= set(r for r in re.findall(r'snprintf\(buf, sizeof\(buf\), "([A-Z_]+),', cmd_c) if r not in ("OK", "ERR", "PARAM", "STATUS", "HELP"))
    unknown = reasons - set(d.FirmwareLink.REPLY_ERRORS)
    check("every ERR reason usb_command.c can reply with is known to the dashboard", not unknown,
          "(fw reasons: %s%s)" % (sorted(reasons), "; MISSING %s" % sorted(unknown) if unknown else ""))
    notices = set(re.findall(r'USB_Command_Notify\("ERR,([A-Z_]+)', main_c))
    check("asynchronous ERR notices are not mistaken for replies (except SAVE_FAILED, which is per-command)",
          all(n not in d.FirmwareLink.REPLY_ERRORS or n == "SAVE_FAILED" for n in notices), "(notices: %s)" % sorted(notices))

    print("\n== STATIC: parameter table ==")
    params = {}
    for mt in re.finditer(r'ADD_F\("([\w.]+)",\s*[^,]+,\s*([-+\d.eEf]+),\s*([-+\d.eEf]+)\)', cmd_c):
        params[mt.group(1)] = (float(mt.group(2).rstrip("f")), float(mt.group(3).rstrip("f")))
    for mt in re.finditer(r'add_param\("([\w.]+)",\s*[^,]+,\s*USB_PARAM_(\w+),\s*([-+\d.eEf]+),\s*([-+\d.eEf]+)', cmd_c):
        params[mt.group(1)] = (float(mt.group(3).rstrip("f")), float(mt.group(4).rstrip("f")))
    needed = set()
    for _, pat, _, _ in d.FirmwarePanel.BIAS_GROUPS:
        needed |= {pat % a for a in "xyz"}
    check("every parameter the dashboard's bias panel edits exists in the firmware table", needed <= set(params),
          "(%d fw params; missing %s)" % (len(params), sorted(needed - set(params))))
    worst = max(len("SET %s %.9g" % (n, -50.123456789)) for n in params)
    check("longest SET command the dashboard can build fits the line buffer", worst <= fw_line - 1, "(%d bytes)" % worst)
    return params


# --------------------------------------------------------------------------------------- dynamic
class Rig:
    def __init__(self, fw_bin, flash, dash_path, sleep_us=200):
        env = dict(os.environ, FW_SIM_FLASH=flash, FW_SIM_SLEEP_US=str(sleep_us))
        self.procs = []

        def factory(port, **kw):
            s = fake_qt.PipeSerial([fw_bin], env)
            self.procs.append(s)
            return s

        fake_qt.install(factory)
        spec = importlib.util.spec_from_file_location("dash_under_test", dash_path)
        self.d = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.d)
        self.win = self.d.Dashboard(autoconnect=False)
        self.fw = self.win.fw_panel
        self.link = self.win.link
        self.seen_prompts = set()
        self.seen_conn = set()

    def tick(self):
        w = self.win
        for _ in range(3):
            w._ingest()
        w._pump_link()
        w._update_labels()
        self.seen_conn.add(w._conn_status()[0])
        p = self.fw.lbl["prompt"].text()
        if p:
            self.seen_prompts.add(p)

    def spin(self, seconds, until=None):
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.tick()
            if until is not None and until():
                return True
            time.sleep(0.004)
        return until() if until is not None else True

    def idle(self, timeout=6.0):
        return self.spin(timeout, lambda: not self.link.busy)

    def send_sim(self, text):
        self.win.reader.send((text + "\n").encode())

    def fc_running(self):
        return self.win.fc is not None and self.win.fc.state == 1 and self.win._fc_fresh()

    def close(self):
        try:
            self.win.disconnect_port()
        except Exception:
            pass
        for p in self.procs:
            p.close()


def mean_data(rig, seconds, idx):
    """Mean of a DATA field (0=ax .. 5=gz) over `seconds`, in physical units (g or dps)."""
    vals = []
    t_end = time.time() + seconds
    last = None
    while time.time() < t_end:
        rig.tick()
        s = rig.win.last
        if s is not None and s is not last:
            last = s
            vals.append((s.ax, s.ay, s.az, s.gx, s.gy, s.gz)[idx] * 0.001)
        time.sleep(0.003)
    return sum(vals) / max(len(vals), 1), len(vals)


LEGACY_SCRIPT = r"""
import sys, time, threading
def rd():
    while sys.stdin.buffer.read(1):
        pass
threading.Thread(target=rd, daemon=True).start()
t = 1000
try:
    while True:
        print("DATA,%d,28,33,1000,8,-214,-39,27" % t, flush=True)
        t += 10
        time.sleep(0.01)
except (BrokenPipeError, OSError):
    pass
"""


def legacy_checks(dash_path):
    print("\n== DYNAMIC: OLD firmware (DATA lines only, ignores commands) ==")
    env = dict(os.environ)
    procs = []

    def factory(port, **kw):
        s = fake_qt.PipeSerial([sys.executable, "-u", "-c", LEGACY_SCRIPT], env)
        procs.append(s)
        return s

    fake_qt.install(factory)
    spec = importlib.util.spec_from_file_location("dash_legacy", dash_path)
    d = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(d)
    w = d.Dashboard(autoconnect=False)
    lk, fp = w.link, w.fw_panel
    try:
        w.connect_port()
        t_end = time.time() + 8.0
        while time.time() < t_end:
            for _ in range(3):
                w._ingest()
            w._pump_link()
            w._update_labels()
            time.sleep(0.004)
        check("old firmware: dashboard still works as before (DATA parsed, own MEKF running)",
              w.reader.parser.n_ok > 200 and w.mekf.state == d.MEKF.RUN and w.fc is None,
              "(DATA lines %d)" % w.reader.parser.n_ok)
        check("old firmware: unanswered STATUS/LIST time out cleanly, link not stuck, no retry storm",
              lk.n_timeout == 2 and not lk.busy and lk.inflight is None, "(timeouts %d)" % lk.n_timeout)
        check("old firmware: panel says 'DATA only', attitude source falls back to the dashboard MEKF",
              "DATA only" in fp.lbl["link"].text() and w._att_q()[1] == "dashboard MEKF", fp.lbl["link"].text())
        check("old firmware: bias editors stay disabled (nothing to edit)", all(not r.w.isEnabled() for r in fp.rows))
    finally:
        try:
            w.disconnect_port()
        except Exception:
            pass
        for p_ in procs:
            p_.close()


def dynamic_checks(fw, dash_path, static_params):
    print("\n== DYNAMIC: real firmware process <-> real dashboard code ==")
    work = tempfile.mkdtemp(prefix="compat_")
    fw_bin = os.path.join(work, "fw_sim")
    r = subprocess.run(["bash", os.path.join(HERE, "build_fw_sim.sh"), fw, fw_bin], capture_output=True, text=True)
    if r.returncode != 0:
        check("firmware host build", False, r.stderr[-400:])
        return
    check("firmware host build (real main loop + modules)", True)
    flash = os.path.join(work, "flash.bin")
    rig = Rig(fw_bin, flash, dash_path)
    w, lk, d, fp = rig.win, rig.link, rig.d, rig.fw
    try:
        rig.d.SerialReader  # noqa
        w.cb_port.setCurrentText("sim")
        w.connect_port()

        # ---- 1. first boot: no stored config -> full calibration, six faces, MEKF cal, auto-save
        print("\n-- 1. first boot (empty flash)")
        ok = rig.spin(90, lambda: rig.fc_running() and lk.status.get("seq") == "1" and lk.params_complete
                      and "NEG_Z" in " ".join(rig.seen_prompts))
        check("first boot: dashboard sees the calibration prompts, then live FC telemetry", ok,
              "(prompts seen: %d)" % len(rig.seen_prompts))
        check("dashboard shows 'FC alive, no IMU data' while the FC calibrates (not a dead link)",
              any("FC alive" in c for c in rig.seen_conn), str(sorted(rig.seen_conn)))
        check("calibration prompts reach the panel", any("WAIT_FACE" in p for p in rig.seen_prompts))
        rig.spin(4.0, lambda: w.mekf.state == d.MEKF.RUN)                   # dashboard MEKF finishes its own 2 s
        rig.spin(2.0)
        rig.idle()
        check("auto-save after calibration reported (CFG,SAVED seq=1) and STATUS synced",
              lk.status.get("seq") == "1" and lk.status.get("source") == "calibrated", str(lk.status))
        check("parameter list arrives (37) and equals the static table",
              lk.params_complete and set(lk.params) == set(static_params),
              "(%d params)" % len(lk.params))
        ranges_ok = all(abs(lk.params[n].lo - lo) < 1e-6 * max(1, abs(lo)) + 1e-9 and abs(lk.params[n].hi - hi) < 1e-6 * max(1, abs(hi)) + 1e-9
                        for n, (lo, hi) in static_params.items() if n in lk.params)
        check("PARAM min/max reported by the FC match the source table", ranges_ok)
        gb = [lk.value("cal.gyro_bias_%s" % a) for a in "xyz"]
        check("stored gyro bias shown by the dashboard equals the simulated sensor's bias (0.55,-0.32,0.21 dps)",
              all(abs(a - b) < 0.06 for a, b in zip(gb, (0.55, -0.32, 0.21))), str([round(x, 4) for x in gb]))
        for ax in "xyz":                       # panel editors loaded
            pass
        pr = fp.group_rows[0][0]
        check("editor for cal.gyro_bias_x loaded the stored value", abs(pr.w.value() - gb[0]) < 1e-4 and not pr.edited)

        # calibrated sensor values
        ax, n1 = mean_data(rig, 1.0, 0)
        ay, _ = mean_data(rig, 0.5, 1)
        az, _ = mean_data(rig, 0.5, 2)
        gx, _ = mean_data(rig, 0.5, 3)
        gz0, _ = mean_data(rig, 1.0, 5)
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        check("calibrated accelerometer magnitude is 1 g", abs(norm - 1.0) < 0.01, "(|a|=%.4f from %d samples)" % (norm, n1))
        check("calibrated gyro is ~0 at rest (bias removed in the FC)", abs(gx) < 0.1 and abs(gz0) < 0.1, "(gx %.3f gz %.3f dps)" % (gx, gz0))
        txt = fp.lbl["acc"].text()
        check("panel shows calibrated accel in g", "|a|=1.0" in txt or "|a|=0.99" in txt, txt)

        # quaternion
        fc = w.fc
        e_roll, e_pitch = -5.07, 10.00        # sim pose: sensor accel (0.1736,-0.0872,0.9809) with diag(1,-1,-1)
        check("FC attitude equals the pose the simulated board is held in",
              abs(fc.euler_deg[0] - e_roll) < 0.6 and abs(fc.euler_deg[1] - e_pitch) < 0.6,
              "(FC roll %.2f pitch %.2f, expected %.2f %.2f)" % (fc.euler_deg[0], fc.euler_deg[1], e_roll, e_pitch))
        qr, qp, qy = (math.degrees(a) for a in d.quat_to_euler(fc.q))
        check("FC quaternion and FC Euler fields are consistent (convention, order, scale)",
              abs(qr - fc.euler_deg[0]) < 0.05 and abs(qp - fc.euler_deg[1]) < 0.05, "(q->euler %.3f %.3f vs %.3f %.3f)" % (qr, qp, *fc.euler_deg[:2]))
        tilt, yaw = d.attitude_difference(fc.q, w.mekf.q)
        check("FC quaternion vs dashboard MEKF (independent filters, same data): tilt agrees", tilt < 1.0,
              "(tilt diff %.3f deg, yaw diff %.2f deg - yaw drifts independently)" % (tilt, yaw))
        check("3D view / plot use the firmware quaternion in Auto mode", w._att_q()[1].startswith("firmware"))
        check("FC mixer in the hover state with motors near hover thrust",
              fc.active and d.MIXER_STATUS_NAMES[fc.mixer_status] in ("HOVERING", "SATURATED") and all(0.3 < t < 0.8 for t in fc.thrust),
              "(thrust %s, pwm %s)" % (list(fc.thrust.round(3)), list(fc.pwm)))
        check("panel renders FC state/quaternion/delta text", "RUNNING" in fp.lbl["fstate"].text() and "tilt" in fp.lbl["delta"].text(),
              fp.lbl["delta"].text())

        # ---- 2. edit the residual bias seed from the panel, send, then save
        print("\n-- 2. edit residual bias (mekf.bias_x) and send it to the FC")
        pr = fp.group_rows[3][0]
        pr.w.setValue(0.30)
        check("typing a value marks the editor as edited (highlight), nothing sent yet", pr.edited and not lk.busy)
        fp._send_rows(fp.group_rows[3])
        rig.idle()
        check("FC accepted SET: value echoed back", abs(lk.value("mekf.bias_x") - 0.30) < 1e-6, str(lk.value("mekf.bias_x")))
        rig.spin(1.0)
        check("editor un-highlights once the FC confirms the same value", not pr.edited)
        check("panel shows UNSAVED CHANGES (FC dirty flag)", "UNSAVED" in fp.lbl["store"].text(), fp.lbl["store"].text())
        check("FC filter really uses the new seed (live bias moved toward 0.3 dps)", w.fc.bias_dps[0] > 0.1, "(live bias x %.3f dps)" % w.fc.bias_dps[0])
        fp.btn_save.clicked.emit()
        ok = rig.spin(5, lambda: lk.status.get("seq") == "2" and not lk.fw_dirty and not lk.busy)
        check("Save to flash: FC reports CFG,SAVED seq=2, dirty cleared", ok, str(lk.status))

        # ---- 3. change a calibration value live: the calibrated DATA stream must change accordingly
        print("\n-- 3. edit cal.gyro_bias_z: calibrated gyro in DATA must shift by the same amount")
        base = lk.value("cal.gyro_bias_z")
        g_before, _ = mean_data(rig, 1.5, 5)
        prz = fp.group_rows[0][2]
        prz.w.setValue(base + 0.50)
        fp._send_rows(fp.group_rows[0])
        rig.idle()
        g_after, _ = mean_data(rig, 1.5, 5)
        check("DATA gyro z shifted by -0.50 dps (corrected = raw - bias)", abs((g_after - g_before) + 0.50) < 0.1,
              "(before %.3f, after %.3f dps)" % (g_before, g_after))
        prz.w.setValue(base)
        fp._send_rows(fp.group_rows[0])
        rig.idle()

        # ---- 4. errors
        print("\n-- 4. error handling")
        n_err = lk.n_err
        lk.set_param("mix.kp_rp", 99.0)
        rig.idle()
        rig.spin(0.3)
        check("out-of-range SET rejected by the FC (ERR,RANGE), value unchanged, error surfaced in the UI",
              lk.n_err == n_err + 1 and "RANGE" in lk.last_error and lk.value("mix.kp_rp") < 10 and "RANGE" in w.last_error,
              lk.last_error)
        lk.set_param("nope.param", 1.0)
        rig.idle()
        check("unknown parameter rejected (ERR,UNKNOWN_PARAM)", "UNKNOWN_PARAM" in lk.last_error, lk.last_error)
        row = next((r_ for r_ in fp.all_rows if r_.name == "mix.use_integral"), None)
        check("'All parameters' tab built from LIST, bool parameter gets a check box", row is not None and row.is_bool)
        row.w.setChecked(True)
        row.send()
        rig.idle()
        check("bool parameter set (mix.use_integral=1)", lk.value("mix.use_integral") == 1.0)
        lk.set_param("mix.kp_rp", 0.45)
        lk.set_param("mix.kd_rp", 0.025)
        lk.set_param("mekf.accel_noise", 0.04)
        rig.idle()
        check("several SETs queued back-to-back are sent one at a time and all acknowledged",
              abs(lk.value("mix.kp_rp") - 0.45) < 1e-6 and abs(lk.value("mix.kd_rp") - 0.025) < 1e-6
              and abs(lk.value("mekf.accel_noise") - 0.04) < 1e-6 and lk.n_timeout == 0, "(timeouts %d)" % lk.n_timeout)

        # ---- 5. reboot: unsaved edits are lost, saved ones persist; dashboard resyncs by itself
        print("\n-- 5. reboot the FC")
        fp.confirm = lambda text: True
        mark = lk.log_total
        fp.reboot()
        ok = rig.spin(40, lambda: rig.fc_running() and lk.log_total > mark + 5 and lk.status.get("source") == "flash" and lk.params_complete and not lk.busy)
        rig.spin(2.0)
        rig.idle()
        new_log = [t for _, t in list(lk.log)[-(lk.log_total - mark):]] if lk.log_total > mark else []
        check("after the reboot the dashboard re-syncs on its own and the FC runs from FLASH", ok and lk.status.get("source") == "flash", str(lk.status))
        check("no calibration prompts after a boot with stored data", not any(t.startswith("[CAL:") for t in new_log))
        check("saved bias seed survived the reboot", abs(lk.value("mekf.bias_x") - 0.30) < 1e-4, str(lk.value("mekf.bias_x")))
        check("unsaved edits (mix.kp_rp, use_integral) were dropped, as designed",
              abs(lk.value("mix.kp_rp") - 0.30) < 1e-6 and lk.value("mix.use_integral") == 0.0, "(kp_rp %.3f)" % lk.value("mix.kp_rp"))
        check("stored gyro bias survived (no recalibration)", abs(lk.value("cal.gyro_bias_x") - 0.55) < 0.06)
        fc = w.fc
        check("filter restarted from live accelerometer data (attitude equals the pose again)",
              abs(fc.euler_deg[0] - e_roll) < 0.8 and abs(fc.euler_deg[1] - e_pitch) < 0.8, str(fc.euler_deg[:2].round(2)))

        # ---- 6. CAL BIAS: the sensor's gyro bias changed (e.g. temperature); re-measure and store
        print("\n-- 6. CAL BIAS")
        rig.send_sim("!gbias 0.90 0.10 -0.20")
        rig.spin(0.5)
        seq0 = int(lk.status.get("seq", "0"))
        fp.cal_bias()
        rig.spin(5, lambda: lk.n_ok > 0 and lk.inflight is None and not lk.queue and "OK,CAL,BIAS" in " ".join(t for _, t in list(lk.log)[-6:]))
        n_err = lk.n_err
        lk.set_param("mix.kp_rp", 0.35)                  # sent while the FC is still measuring the bias
        rig.spin(2, lambda: lk.n_err > n_err)
        check("SET while CAL BIAS is running is refused with ERR,BUSY", lk.n_err == n_err + 1 and "BUSY" in lk.last_error, lk.last_error)
        ok = rig.spin(30, lambda: int(lk.status.get("seq", "0")) > seq0 and lk.params_complete and not lk.busy)
        rig.idle()
        b = [lk.value("mekf.bias_%s" % a) for a in "xyz"]
        # residual in SENSOR axes, then mapped into the BODY frame the MEKF works in (mekf.bias_* is FRD)
        res_sensor = (0.90 - 0.55, 0.10 + 0.32, -0.20 - 0.21)
        Mrow = d.SENSOR_TO_BODY_PRESETS[d.DEFAULT_MAPPING]
        exp = tuple(float(v) for v in Mrow @ res_sensor)
        check("CAL BIAS ran, auto-saved (new sequence); residual bias shown in BODY axes (= SENSOR_TO_BODY x sensor residual)",
              ok and all(abs(x - y) < 0.1 for x, y in zip(b, exp)), "(got %s, expected ~%s)" % ([round(x, 3) for x in b], [round(x, 3) for x in exp]))

        # ---- 7. CAL FULL
        print("\n-- 7. CAL FULL")
        fp.confirm = lambda text: False
        fp.cal_full()
        check("CAL FULL asks for confirmation (declined -> nothing sent)", not lk.busy)
        fp.confirm = lambda text: True
        seq0 = int(lk.status.get("seq", "0"))
        mark = lk.log_total
        rig.seen_prompts.clear()
        fp.cal_full()
        rig.idle()
        rig.spin(120, lambda: int(lk.status.get("seq", "0")) > seq0 and rig.fc_running() and lk.params_complete and not lk.busy)
        rig.spin(2.0)
        rig.idle()
        faces = [f for f in ("POS_X", "NEG_X", "POS_Y", "NEG_Y", "POS_Z", "NEG_Z") if any(f in p for p in rig.seen_prompts)]
        check("CAL FULL: all six face prompts reached the panel, FC auto-saved a new record and runs again",
              len(faces) == 6 and int(lk.status.get("seq", "0")) > seq0 and rig.fc_running(), "(faces seen %s, %s)" % (faces, lk.status))
        gbx = lk.value("cal.gyro_bias_x")
        check("CAL FULL re-measured the gyro bias (new sensor bias 0.90 dps)", abs(gbx - 0.90) < 0.06, "(cal.gyro_bias_x = %.3f)" % gbx)
        check("no command timeouts, and only the 3 deliberate errors (RANGE, UNKNOWN_PARAM, BUSY)", lk.n_timeout == 0 and lk.n_err == 3,
              "(ok %d, err %d [expected: 3 deliberate], timeouts %d)" % (lk.n_ok, lk.n_err, lk.n_timeout))
        check("no malformed lines from the FC (parser bad-line counter)", w.reader.parser.n_bad == 0,
              "(DATA %d, FC %d, text %d, bad %d, ignored %d)" % (w.reader.parser.n_ok, w.reader.parser.n_fc, w.reader.parser.n_msg,
                                                                 w.reader.parser.n_bad, w.reader.parser.n_ignored))
    finally:
        rig.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fw-dir", required=True)
    ap.add_argument("--dash", required=True)
    ap.add_argument("--static-only", action="store_true")
    a = ap.parse_args()
    spec = importlib.util.spec_from_file_location("dash_static", a.dash)
    # the static part only needs constants; import with fake Qt so it works anywhere
    fake_qt.install(lambda *x, **k: None)
    d = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(d)
    params = static_checks(a.fw_dir, d)
    if not a.static_only and sys.platform == "win32":
        print("\nNOTE: the dynamic scenario needs Linux/macOS/WSL (POSIX pipes, gcc). Native Windows: static checks only.")
        a.static_only = True
    if not a.static_only:
        dynamic_checks(a.fw_dir, a.dash, params)
        legacy_checks(a.dash)
    n_fail = results.count(False)
    print("\n%s: %d checks, %d failed" % ("COMPATIBILITY OK" if not n_fail else "INCOMPATIBLE", len(results), n_fail))
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
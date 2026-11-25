#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SORA-FC Attitude Monitoring Dashboard
=====================================
Real-time 3D drone orientation from an STM32 flight controller over USB CDC.

Install:   pip install numpy pyserial pyqtgraph PyQt6 PyOpenGL
Run:       python sora_orientation_dashboard.py [--port COM10] [--baud 115200]
Self-test: python sora_orientation_dashboard.py --selftest      (headless, no GUI needed)

Telemetry:  DATA,timestamp_ms,Ax,Ay,Az,Gx,Gy,Gz[,extra...]   CALIBRATED values (mg, mdps) from the FC
            FC,timestamp_ms,...27 integer fields                firmware quaternion/bias/motor outputs
            OK / ERR / PARAM / STATUS / CFG / CAL / WARN lines   replies and notices of the command link
Commands (sent to the FC over the same CDC port): STATUS, LIST, GET/SET <name> [value], SAVE, LOAD,
            CAL BIAS, CAL FULL, CLEAR, REBOOT   -- see usb_command.h. The 'Firmware' dock drives them.

COORDINATE CONVENTIONS (used consistently everywhere)
-----------------------------------------------------
* Body frame  (b): aerospace FRD  -> x forward (nose), y right, z down.
* World frame (w): NED            -> x north, y east,  z down.  Gravity = +z_w.
* Quaternion: Hamilton, ordering [w, x, y, z], unit norm.
  q rotates BODY -> WORLD:   v_w = R(q) v_b.   Product q = a (x) b means "apply b, then a".
* Euler angles (display only): aerospace ZYX (yaw, pitch, roll); roll about x, pitch about y,
  yaw about z. +roll = right wing down, +pitch = nose up, +yaw = nose to the right (clockwise
  seen from above).
* Accelerometer: specific force. At rest and level, FRD body reads [0, 0, -1 g].
* Sensor -> body: the telemetry sensor frame is NOT assumed to equal the body frame. A 3x3
  matrix SENSOR_TO_BODY (selectable in the GUI) maps sensor axes into FRD. Default assumes a
  sensor frame x-forward, y-left, z-up (this is what your sample data implies: Az ~ +1000 when
  level), i.e. diag(1,-1,-1). CHANGE IT if your board is mounted differently.
* Units: accel counts * ACCEL_G_PER_LSB -> g (default 0.001, i.e. mg);
         gyro counts * GYRO_DPS_PER_LSB -> deg/s (default 0.001: the FC sends milli-deg/s).
* 3D view: OpenGL display frame is x north, y west, z up = diag(1,-1,-1) applied to NED, a pure
  rotation (det=+1), so no mirroring. Model vertices are authored in FRD body coordinates and
  drawn with the matrix  T_ned2display @ R(q).

MEKF
----
Nominal state q (body->world), gyro bias bg [rad/s]. Error state dx = [dtheta(3), dbg(3)] with a
body-frame (right-multiplicative) attitude error: q_true = q_hat (x) exp(dtheta), bg_true = bg_hat + dbg.
  Predict:  omega = gyro - bg;  q <- q (x) exp(omega dt);
            P <- F P F^T + Q,  F = [[exp(-[omega dt]x), -I dt],[0, I]]  (first-order in the bias block)
  Update:   unit accelerometer vector z vs. h(q) = -R(q)^T e3;  H = [[h]x, 0];
            Joseph-form covariance update, error injection q <- q (x) exp(dtheta), bg += dbg,
            covariance reset P <- G P G^T with G = diag(I - 0.5[dtheta]x, I).
  Accelerometer measurements are rejected if ||a|| deviates > accel_reject_tol from gravity, and
  their noise is inflated smoothly for smaller deviations (dynamic acceleration).
  Yaw is unobservable without a magnetometer: it is integrated from the gyro and WILL DRIFT.
  All noise parameters in MEKFConfig are sensible starting values and MUST BE TUNED on hardware.
"""

import os
import sys
import math
import time
import queue
import argparse
import threading
import collections
from dataclasses import dataclass

import numpy as np

os.environ.setdefault("PYQTGRAPH_QT_LIB", "PyQt6")

try:
    import serial
    import serial.tools.list_ports
    HAVE_SERIAL = True
except ImportError:  # pragma: no cover
    serial = None
    HAVE_SERIAL = False

try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
    HAVE_GUI = True
    GUI_IMPORT_ERROR = ""
except Exception as _exc:  # pragma: no cover
    pg = QtCore = QtGui = QtWidgets = None
    HAVE_GUI = False
    GUI_IMPORT_ERROR = str(_exc)

try:
    import pyqtgraph.opengl as gl
    HAVE_GL = HAVE_GUI
    GL_IMPORT_ERROR = ""
except Exception as _exc:  # pragma: no cover
    gl = None
    HAVE_GL = False
    GL_IMPORT_ERROR = str(_exc)


# ----------------------------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------------------------
DEFAULT_PORT = "COM10"
DEFAULT_BAUD = 115200
BAUD_CHOICES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 2000000]

ACCEL_G_PER_LSB = 0.001        # accel counts -> g (mg per LSB)
GYRO_DPS_PER_LSB = 0.001       # gyro counts are mdps (milli-deg/s) -> deg/s

SENSOR_TO_BODY_PRESETS = collections.OrderedDict([
    ("Sensor x fwd, y left, z up  -> FRD", np.diag([1.0, -1.0, -1.0])),
    ("Sensor already FRD (identity)", np.eye(3)),
    ("Sensor x right, y fwd, z up -> FRD", np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])),
])
DEFAULT_MAPPING = next(iter(SENSOR_TO_BODY_PRESETS))

NED_TO_DISPLAY = np.diag([1.0, -1.0, -1.0])   # NED (x N, y E, z D) -> view (x N, y W, z Up)

STALE_TIMEOUT_S = 1.0
HISTORY_SECONDS = 15.0
MAX_HISTORY_RATE_HZ = 600


# ----------------------------------------------------------------------------------------------
# Quaternion / rotation helpers (all double precision)
# ----------------------------------------------------------------------------------------------
_IDENT_Q = np.array([1.0, 0.0, 0.0, 0.0])


def skew(v):
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_normalize(q):
    n = math.sqrt(float(q @ q))
    if not math.isfinite(n) or n < 1e-12:
        return _IDENT_Q.copy()
    return q / n


def quat_from_rotvec(v):
    """Exponential map: rotation vector (rad) -> unit quaternion."""
    th = math.sqrt(float(v @ v))
    if th < 1e-9:
        return quat_normalize(np.array([1.0, 0.5 * v[0], 0.5 * v[1], 0.5 * v[2]]))
    s = math.sin(0.5 * th) / th
    return np.array([math.cos(0.5 * th), s * v[0], s * v[1], s * v[2]])


def quat_to_rot(q):
    """Body->world rotation matrix of a unit quaternion [w,x,y,z]."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rotvec_to_rot(v):
    """Rodrigues formula with a small-angle series (stable near zero)."""
    th2 = float(v @ v)
    K = skew(v)
    if th2 < 1e-12:
        return np.eye(3) + K + 0.5 * (K @ K)
    th = math.sqrt(th2)
    return np.eye(3) + (math.sin(th) / th) * K + ((1.0 - math.cos(th)) / th2) * (K @ K)


def quat_to_euler(q):
    """Aerospace ZYX Euler angles (roll, pitch, yaw) in radians. Display only."""
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return quat_normalize(np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]))


def quat_from_accel(a):
    """Level attitude (yaw = 0) from a specific-force vector in FRD body axes."""
    n = float(np.linalg.norm(a))
    if n < 1e-9 or not math.isfinite(n):
        return _IDENT_Q.copy()
    a = a / n
    roll = math.atan2(-a[1], -a[2])
    pitch = math.asin(max(-1.0, min(1.0, a[0])))
    return euler_to_quat(roll, pitch, 0.0)


# ----------------------------------------------------------------------------------------------
# Telemetry parsing
#
# The firmware (see usb_telemetry.h / usb_command.h) sends, at 100 Hz, one packet holding two lines:
#   DATA,tick,Ax,Ay,Az,Gx,Gy,Gz,Temp    CALIBRATED sensor values: accel in mg, gyro in mdps
#   FC,tick,<26 more integer fields>      firmware attitude filter + motor mixer (see parse_fc)
# and, on demand or when something happens, plain text lines:
#   OK,... / ERR,... / PARAM,... / STATUS,... / CFG,... / CAL,... / WARN,... / HELP,... / [CAL:...]
# ----------------------------------------------------------------------------------------------
Sample = collections.namedtuple("Sample", "t ax ay az gx gy gz extra")

# Firmware attitude filter + mixer record. Units are already converted (deg, dps, 0..1, us).
FcRecord = collections.namedtuple(
    "FcRecord", "t state event q euler_deg bias_dps mixer_status active saturated thrust pwm collective torque")

# Any other firmware text line. kind is one of FW_LINE_KINDS or "CALPROMPT".
FwLine = collections.namedtuple("FwLine", "kind fields raw")

FC_FIELD_COUNT = 27     # must match the field list in usb_telemetry.h

# These tables must stay in the same order as the enums in mekf.h / motor_mixer.h
# (compat_check.py verifies this against the firmware headers).
MEKF_STATE_NAMES = ("CALIBRATING", "RUNNING")
MEKF_EVENT_NAMES = ("NONE", "CALIBRATING", "CALIBRATION_RESTART", "RUNNING", "RUNNING_CALIBRATED_WITH_MOTION",
                    "ACCEL_DOWNWEIGHTED", "ACCEL_INVALID", "ACCEL_REJECTED", "ACCEL_SINGULAR",
                    "ACCEL_INNOVATION_REJECTED", "DATA_GAP", "ATTITUDE_RESET", "INVALID_SAMPLE",
                    "NUMERICAL_RESET")
MIXER_STATUS_NAMES = ("DISABLED", "WAITING_FOR_FILTER", "INVALID_INPUT", "TILT_LIMIT", "HOVERING", "SATURATED")
FW_LINE_KINDS = ("OK", "ERR", "PARAM", "STATUS", "CFG", "CAL", "WARN", "HELP")


def _name(table, i):
    return table[i] if 0 <= i < len(table) else "#%d" % i


class TelemetryParser:
    """Incremental line parser: tolerates partial reads, multiple packets per read, garbage."""

    MAX_BUFFER = 8192

    def __init__(self):
        self._buf = bytearray()
        self.n_ok = 0          # DATA lines
        self.n_fc = 0          # FC lines
        self.n_msg = 0         # other firmware text lines
        self.n_bad = 0
        self.n_ignored = 0

    @staticmethod
    def parse_line(line):
        """Return a Sample, None if the line is not a DATA record, raise ValueError if malformed."""
        parts = line.strip().split(",")
        if parts[0].strip() != "DATA":
            return None
        if len(parts) < 8:
            raise ValueError("too few fields")
        vals = [float(p) for p in parts[1:8]]
        for v in vals:
            if not math.isfinite(v):
                raise ValueError("non-finite value")
        if vals[0] < 0:
            raise ValueError("negative timestamp")
        extra = []
        for p in parts[8:]:
            try:
                extra.append(float(p))
            except ValueError:
                extra.append(p.strip())
        return Sample(int(vals[0]), *vals[1:7], extra)

    @staticmethod
    def parse_fc(parts):
        """FC,tick,state,event,roll,pitch,yaw,qw,qx,qy,qz,bx,by,bz,mstatus,mflags,
              t1..t4,pwm1..pwm4,collective,torque_r,torque_p,torque_y   (all integers)"""
        if len(parts) < 1 + FC_FIELD_COUNT:
            raise ValueError("FC: too few fields (%d)" % (len(parts) - 1))
        v = [int(p) for p in parts[1:1 + FC_FIELD_COUNT]]
        if v[0] < 0:
            raise ValueError("FC: negative timestamp")
        q = np.array(v[6:10], float) / 1e4
        n = float(np.linalg.norm(q))
        if abs(n - 1.0) > 0.02:
            raise ValueError("FC: quaternion not unit (|q|=%.3f)" % n)
        return FcRecord(
            t=v[0], state=v[1], event=v[2], q=q / n,
            euler_deg=np.array(v[3:6], float) / 1000.0,
            bias_dps=np.array(v[10:13], float) / 1000.0,
            mixer_status=v[13], active=bool(v[14] & 1), saturated=bool(v[14] & 2),
            thrust=np.array(v[15:19], float) / 1000.0,
            pwm=np.array(v[19:23], float),
            collective=v[23] / 1000.0,
            torque=np.array(v[24:27], float) / 1000.0)

    @classmethod
    def parse_any(cls, line):
        """Sample | FcRecord | FwLine | None (unknown line). Raises ValueError for malformed DATA/FC."""
        s = line.strip()
        if s.startswith("[CAL:"):
            return FwLine("CALPROMPT", [s], s)
        parts = s.split(",")
        head = parts[0].strip()
        if head == "DATA":
            return cls.parse_line(line)
        if head == "FC":
            return cls.parse_fc(parts)
        if head in FW_LINE_KINDS:
            return FwLine(head, [p.strip() for p in parts], s)
        return None

    def _feed(self, data, typed):
        out = []
        self._buf.extend(data.replace(b"\r", b"\n"))
        while True:
            i = self._buf.find(b"\n")
            if i < 0:
                break
            raw = bytes(self._buf[:i])
            del self._buf[:i + 1]
            if not raw.strip():
                continue
            text = raw.decode("ascii", errors="ignore")
            try:
                s = self.parse_any(text) if typed else self.parse_line(text)
            except ValueError:
                self.n_bad += 1
                continue
            if s is None:
                self.n_ignored += 1
            elif isinstance(s, Sample):
                self.n_ok += 1
                out.append(s)
            elif isinstance(s, FcRecord):
                self.n_fc += 1
                out.append(s)
            else:
                self.n_msg += 1
                out.append(s)
        if len(self._buf) > self.MAX_BUFFER:   # runaway garbage without newline
            self._buf.clear()
            self.n_bad += 1
        return out

    def feed(self, data):
        """DATA samples only (original behaviour)."""
        return self._feed(data, False)

    def feed_all(self, data):
        """Samples, FcRecords and FwLines, in the order they arrived."""
        return self._feed(data, True)


class TimeBase:
    """Timestamp -> dt, with rollover and discontinuity detection."""

    ROLLOVER_MODULI = (2 ** 32, 2 ** 16)

    def __init__(self, max_gap_s=0.1):
        self.max_gap_s = max_gap_s
        self.reset()

    def reset(self):
        self.last = None
        self.gaps = 0
        self.rollovers = 0
        self.duplicates = 0
        self.resets = 0

    def update(self, t_ms):
        """Return (dt_s or None, kind, gap_s). kind: first/ok/rollover/gap/duplicate/reset."""
        if self.last is None:
            self.last = t_ms
            return None, "first", 0.0
        diff = t_ms - self.last
        kind = "ok"
        if diff < 0:
            for mod in self.ROLLOVER_MODULI:
                d2 = diff + mod
                if 0 < d2 <= self.max_gap_s * 1000.0:
                    diff = d2
                    kind = "rollover"
                    self.rollovers += 1
                    break
            else:
                self.last = t_ms          # FC restarted or timestamps jumped backwards
                self.resets += 1
                return None, "reset", 0.0
        if diff == 0:
            self.duplicates += 1
            return None, "duplicate", 0.0
        self.last = t_ms
        dt = diff / 1000.0
        if dt > self.max_gap_s:
            self.gaps += 1
            return None, "gap", dt
        return dt, kind, 0.0


# ----------------------------------------------------------------------------------------------
# Firmware command link (host side of usb_command.c)
#
# Protocol summary (text lines, CR/LF terminated, case-insensitive on the firmware side):
#   STATUS | LIST [prefix] | GET <name> | SET <name> <value> | SAVE | LOAD | CLEAR | REBOOT
#   CAL FULL | CAL BIAS
# The firmware parses ONE command per main-loop pass and only after every queued reply has been
# transmitted, so the host must also send ONE command at a time and wait for its reply. Replies are
# "OK,..." or "ERR,<reason>,..."; LIST answers with PARAM lines then "OK,LIST,<n>".
# Some "ERR,..." lines are NOT replies (asynchronous notices such as ERR,CAL_BIAS,motion...), which
# is why a reply is only accepted if its reason is one the command handler can return.
# ----------------------------------------------------------------------------------------------
@dataclass
class FwParam:
    name: str
    value: float
    lo: float
    hi: float
    is_bool: bool = False
    rev: int = 0               # incremented every time the firmware reports a value for this name


class FwCommand:
    __slots__ = ("kind", "text", "timeout", "sent_at", "on_done", "arg")

    def __init__(self, kind, text, timeout, on_done=None, arg=None):
        self.kind, self.text, self.timeout, self.on_done, self.arg = kind, text, timeout, on_done, arg
        self.sent_at = 0.0


class FirmwareLink:
    # Error reasons usb_command.c can return as the REPLY to a command, per command. A reply is only
    # accepted if the command in flight can produce that reason: the firmware also sends unsolicited
    # "ERR,..." notices (e.g. ERR,SAVE_FAILED after an automatic save, ERR,CAL_BIAS,motion...).
    _COMMON_ERRORS = ("UNKNOWN_CMD", "UNSUPPORTED", "TOO_MANY_ARGS", "LINE_TOO_LONG", "USAGE")
    ERRORS_BY_KIND = {
        "GET": ("UNKNOWN_PARAM",),
        "SET": ("UNKNOWN_PARAM", "BUSY", "BAD_VALUE", "RANGE"),
        "SAVE": ("BUSY", "SAVE_FAILED"),
        "LOAD": ("BUSY", "NO_STORED_CONFIG"),
        "CLEAR": ("CLEAR_FAILED",),
        "CAL_FULL": ("CAL_FULL_REFUSED",),
        "CAL_BIAS": ("CAL_BIAS_REFUSED",),
    }
    REPLY_ERRORS = frozenset(_COMMON_ERRORS) | frozenset(r for v in ERRORS_BY_KIND.values() for r in v)
    # Second token of the "OK,..." reply for each command kind.
    OK_TOKEN = {"STATUS": None, "LIST": "LIST", "GET": "GET", "SET": "SET", "SAVE": "SAVE", "LOAD": "LOAD",
                "CLEAR": "CLEAR", "REBOOT": "REBOOT", "CAL_BIAS": "CAL", "CAL_FULL": "CAL"}
    TIMEOUT_S = {"SAVE": 4.0, "CLEAR": 6.0, "LIST": 3.0}
    DEFAULT_TIMEOUT_S = 1.5
    MAX_QUEUE = 64
    MAX_LINE = 95                       # firmware line buffer is 96 bytes including the terminator

    def __init__(self, send_fn):
        self.send_fn = send_fn           # callable(bytes) -> bool
        self.log = collections.deque(maxlen=500)
        self.log_total = 0
        self.reset()

    # -- state -------------------------------------------------------------------------------
    def reset(self):
        self.queue = collections.deque()
        self.inflight = None
        self.params = collections.OrderedDict()
        self.params_complete = False
        self.param_rev = 0
        self.status = {}
        self.status_rev = 0
        self.fw_dirty = False
        self.cal_prompt = ""
        self.last_notice = ""
        self.n_ok = 0
        self.n_err = 0
        self.n_timeout = 0
        self.last_error = ""

    def _log(self, text, now=None):
        self.log.append((time.time() if now is None else now, text))
        self.log_total += 1

    # -- sending -----------------------------------------------------------------------------
    def submit(self, kind, text, on_done=None, arg=None, dedupe=False):
        if len(text) > self.MAX_LINE:
            self._log("host: command too long, not sent: %s" % text[:40])
            return False
        if dedupe and any(c.kind == kind and c.text == text for c in self.queue):
            return True
        if len(self.queue) >= self.MAX_QUEUE:
            self._log("host: command queue full, dropped: %s" % text)
            return False
        self.queue.append(FwCommand(kind, text, self.TIMEOUT_S.get(kind, self.DEFAULT_TIMEOUT_S), on_done, arg))
        return True

    @property
    def busy(self):
        return self.inflight is not None or bool(self.queue)

    def request_sync(self):
        """Ask for STATUS and the full parameter list (after connect / reboot / calibration)."""
        self.submit("STATUS", "STATUS", dedupe=True)
        self.submit("LIST", "LIST", dedupe=True)

    def get(self, name):
        return self.submit("GET", "GET %s" % name, arg=name)

    def set_param(self, name, value, on_done=None):
        if isinstance(value, bool):
            txt = "1" if value else "0"
        else:
            value = float(value)
            if not math.isfinite(value):
                self._log("host: refusing to send non-finite value for %s" % name)
                return False
            txt = "%.9g" % value
        return self.submit("SET", "SET %s %s" % (name, txt), on_done=on_done, arg=name)

    def save(self):      return self.submit("SAVE", "SAVE")
    def load(self):
        ok = self.submit("LOAD", "LOAD")
        self.request_sync()
        return ok
    def cal_bias(self):  return self.submit("CAL_BIAS", "CAL BIAS")
    def cal_full(self):  return self.submit("CAL_FULL", "CAL FULL")
    def clear(self):     return self.submit("CLEAR", "CLEAR")
    def reboot(self):    return self.submit("REBOOT", "REBOOT")

    def pump(self, now=None):
        """Call every ~20 ms: expire a stuck command, then send the next one."""
        now = time.monotonic() if now is None else now
        c = self.inflight
        if c is not None and now - c.sent_at > c.timeout:
            self.n_timeout += 1
            self.last_error = "no reply to '%s' (timeout)" % c.text
            self._log("host: TIMEOUT waiting for reply to '%s'" % c.text)
            self._finish(False, "TIMEOUT")
        if self.inflight is None and self.queue:
            nxt = self.queue.popleft()
            if self.send_fn((nxt.text + "\n").encode("ascii")):
                nxt.sent_at = now
                self.inflight = nxt
            else:
                self._log("host: not connected, dropped: %s" % nxt.text)
                self.queue.clear()

    def _finish(self, ok, msg):
        c, self.inflight = self.inflight, None
        if c is None:
            return
        if ok:
            self.n_ok += 1
        else:
            self.n_err += 1
            self.last_error = "%s: %s" % (c.text, msg)
        if c.on_done is not None:
            try:
                c.on_done(ok, msg)
            except Exception:       # a UI callback must never break the protocol
                pass

    # -- receiving ---------------------------------------------------------------------------
    def on_line(self, ln):
        """Handle one FwLine from the firmware."""
        k, f = ln.kind, ln.fields
        self._log(ln.raw)

        if k == "PARAM":
            self._on_param(f)
        elif k == "STATUS":
            self._on_status(f)
            if self.inflight is not None and self.inflight.kind == "STATUS":
                self._finish(True, "")
        elif k == "OK":
            self._on_ok(f)
        elif k == "ERR":
            reason = f[1] if len(f) > 1 else ""
            c = self.inflight
            if c is not None and (reason in self._COMMON_ERRORS or reason in self.ERRORS_BY_KIND.get(c.kind, ())):
                self._finish(False, ",".join(f[1:]))
            else:
                self.last_notice = ln.raw        # asynchronous notice
        elif k == "CALPROMPT":
            self.cal_prompt = ln.raw
        elif k == "CAL":
            self.last_notice = ln.raw
            if len(f) > 1 and f[1] in ("IDLE", "COMPLETE", "ERROR"):
                self.cal_prompt = ln.raw
        elif k == "CFG":
            self.last_notice = ln.raw
            what = f[1] if len(f) > 1 else ""
            if what == "SAVED":
                self.fw_dirty = False
            if what in ("SAVED", "LOADED", "EMPTY"):
                self.request_sync()               # stored values may have changed (calibration finished)
        elif k == "WARN":
            self.last_notice = ln.raw

    def _on_param(self, f):
        # PARAM,name,value,min,max[,bool]
        try:
            name = f[1]
            val, lo, hi = float(f[2]), float(f[3]), float(f[4])
        except (IndexError, ValueError):
            return
        p = self.params.get(name)
        is_bool = len(f) > 5 and f[5] == "bool"
        if p is None:
            self.params[name] = FwParam(name, val, lo, hi, is_bool, 1)
        else:
            p.value, p.lo, p.hi, p.is_bool = val, lo, hi, is_bool
            p.rev += 1
        self.param_rev += 1

    def _on_status(self, f):
        d = {}
        for item in f[1:]:
            if "=" in item:
                key, _, val = item.partition("=")
                d[key] = val
        self.status.update(d)
        self.status_rev += 1
        if "dirty" in d:
            self.fw_dirty = (d["dirty"] == "1")

    def _on_ok(self, f):
        tok = f[1] if len(f) > 1 else ""
        c = self.inflight
        if tok in ("SET", "GET") and len(f) >= 4:
            try:
                val = float(f[3])
            except ValueError:
                val = None
            p = self.params.get(f[2])
            if val is not None:
                if p is None:
                    self.params[f[2]] = FwParam(f[2], val, -math.inf, math.inf, False, 1)
                else:
                    p.value = val
                    p.rev += 1
                self.param_rev += 1
            if tok == "SET" and len(f) >= 5 and f[4] == "UNSAVED":
                self.fw_dirty = True
        elif tok == "SAVE":
            self.fw_dirty = False
        elif tok == "LOAD":
            self.fw_dirty = False
        elif tok == "LIST" and len(f) >= 3:
            self.params_complete = True
        if c is not None and self.OK_TOKEN.get(c.kind) == tok:
            self._finish(True, ",".join(f[1:]))

    # -- convenience for the UI --------------------------------------------------------------
    def value(self, name, default=float("nan")):
        p = self.params.get(name)
        return p.value if p is not None else default

    def new_log(self, since):
        """Log entries added after index `since` (absolute count). Returns (entries, new_since)."""
        n_new = self.log_total - since
        if n_new <= 0:
            return [], self.log_total
        n_new = min(n_new, len(self.log))
        return list(self.log)[-n_new:], self.log_total


# ----------------------------------------------------------------------------------------------
# Serial reader thread
# ----------------------------------------------------------------------------------------------
class SerialReader(threading.Thread):
    CONNECTING, CONNECTED, LOST, ERROR, STOPPED = "CONNECTING", "CONNECTED", "LOST", "ERROR", "STOPPED"

    def __init__(self, port, baud, out_queue):
        super().__init__(name="SerialReader", daemon=True)
        self.port, self.baud, self.q = port, baud, out_queue
        self._stop_evt = threading.Event()
        self.parser = TelemetryParser()
        self.state = self.CONNECTING
        self.error = ""
        self.ser = None
        self.last_rx = 0.0            # monotonic time of the last valid DATA packet
        self.last_any_rx = 0.0        # ... of any valid firmware line (DATA, FC, replies, notices)
        self.valid_count = 0
        self.dropped = 0
        self._tx_lock = threading.Lock()

    def stop(self):
        self._stop_evt.set()

    def send(self, data):
        """Write bytes to the flight controller (thread-safe). Returns False if not connected."""
        ser = self.ser
        if ser is None or self.state != self.CONNECTED:
            return False
        try:
            with self._tx_lock:
                ser.write(data)
            return True
        except Exception as exc:
            self.error = "Write failed: %s" % exc
            return False

    def run(self):
        try:
            self.ser = serial.serial_for_url(self.port, baudrate=self.baud, timeout=0.05, write_timeout=0.5)
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            self.state = self.CONNECTED
        except Exception as exc:
            self.error = "Cannot open %s: %s" % (self.port, exc)
            self.state = self.ERROR
            return
        try:
            while not self._stop_evt.is_set():
                n = self.ser.in_waiting
                data = self.ser.read(min(max(n, 1), 4096))      # blocks <= timeout: no busy loop
                if not data:
                    continue
                for s in self.parser.feed_all(data):
                    now = time.monotonic()
                    self.last_any_rx = now
                    if isinstance(s, Sample):
                        self.valid_count += 1
                        self.last_rx = now
                    try:
                        self.q.put_nowait(s)
                    except queue.Full:
                        try:
                            self.q.get_nowait()
                            self.dropped += 1
                        except queue.Empty:
                            pass
                        try:
                            self.q.put_nowait(s)
                        except queue.Full:
                            self.dropped += 1
            self.state = self.STOPPED
        except Exception as exc:        # SerialException, OSError (USB unplugged), ...
            if not self._stop_evt.is_set():
                self.error = "Connection lost: %s" % exc
                self.state = self.LOST
            else:
                self.state = self.STOPPED
        finally:
            try:
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass


# ----------------------------------------------------------------------------------------------
# MEKF
# ----------------------------------------------------------------------------------------------
@dataclass
class MEKFConfig:
    calib_duration_s: float = 2.0        # stationary gyro-bias / gravity calibration time
    calib_max_gyro_std: float = 0.15     # rad/s: more motion than this restarts calibration
    gyro_noise: float = 0.01             # rad/s/sqrt(Hz)  (continuous-time angular random walk)
    gyro_bias_rw: float = 1e-4           # rad/s/sqrt(s)   (bias random walk)
    accel_noise: float = 0.05            # std of the unit gravity-direction measurement
    init_att_std: float = 0.05           # rad, after accelerometer initialisation
    init_bias_std: float = 0.01          # rad/s
    gravity: float = 1.0                 # expected |a| in the units fed to the filter (g)
    accel_soft_tol: float = 0.05         # relative |a| error where R starts to be inflated
    accel_reject_tol: float = 0.25       # relative |a| error above which the update is skipped
    max_cov: float = 10.0                # clamp for covariance diagonal


class MEKF:
    CALIB, RUN = "CALIBRATING", "RUNNING"

    def __init__(self, cfg=None):
        self.cfg = cfg or MEKFConfig()
        self.reinitialize()

    # -- lifecycle --------------------------------------------------------------------------
    def reinitialize(self):
        self.q = _IDENT_Q.copy()
        self.bg = np.zeros(3)
        self._init_cov()
        self.state = self.CALIB
        self.rejected = 0
        self.updates = 0
        self.recoveries = 0
        self.last_acc = None
        self.msg = "Calibrating - keep the drone still"
        self._start_calibration(bias_only=False)
        self._q_prev = self.q.copy()

    def _init_cov(self):
        c = self.cfg
        self.P = np.diag([c.init_att_std ** 2] * 3 + [c.init_bias_std ** 2] * 3).astype(float)
        self.P[:3, :3] = np.eye(3) * (0.5 ** 2)    # unknown attitude until calibration completes

    def _start_calibration(self, bias_only):
        self._bias_only = bias_only
        self._cn = 0
        self._cel = 0.0
        self._sg = np.zeros(3)
        self._sg2 = np.zeros(3)
        self._sa = np.zeros(3)
        self._tries = 0
        self.state = self.CALIB

    def recalibrate_bias(self):
        """Re-estimate gyro bias only (attitude kept). Drone must be stationary."""
        self._start_calibration(bias_only=True)
        self.msg = "Re-calibrating gyro bias - keep still"

    def reset_attitude(self):
        """Re-level to the current accelerometer reading with yaw = 0; bias is kept."""
        if self.last_acc is not None:
            self.q = quat_from_accel(self.last_acc)
        else:
            self.q = _IDENT_Q.copy()
        self.P[:3, :3] = np.eye(3) * self.cfg.init_att_std ** 2
        self.P[:3, 3:] = 0.0
        self.P[3:, :3] = 0.0
        self._q_prev = self.q.copy()
        self.msg = "Attitude reset"

    # -- main entry --------------------------------------------------------------------------
    def process(self, gyro, acc, dt):
        """gyro [rad/s] and acc [g] in FRD body axes; dt [s] > 0. Returns a status string."""
        if not (np.all(np.isfinite(gyro)) and np.all(np.isfinite(acc)) and dt > 0.0 and math.isfinite(dt)):
            self.msg = "Invalid sample ignored"
            return self.msg
        self.last_acc = acc
        if self.state == self.CALIB:
            self._calibrate(gyro, acc, dt)
            return self.msg
        self._predict(gyro, dt)
        self._update_acc(acc)
        self._finish()
        return self.msg

    def inflate(self, gap_s):
        """Add process noise for a data gap instead of propagating on a stale/invalid dt."""
        self.P = self.P + self._Q(min(gap_s, 0.5))
        self._sanitize_cov()
        self.msg = "Data gap %.0f ms - covariance inflated" % (gap_s * 1000)

    # -- calibration -------------------------------------------------------------------------
    def _calibrate(self, gyro, acc, dt):
        self._cn += 1
        self._cel += dt
        self._sg += gyro
        self._sg2 += gyro * gyro
        self._sa += acc
        if self._cel < self.cfg.calib_duration_s:
            self.msg = "Calibrating %d%% - keep the drone still" % int(100 * self._cel / self.cfg.calib_duration_s)
            return
        n = self._cn
        mg = self._sg / n
        std = float(np.max(np.sqrt(np.maximum(self._sg2 / n - mg * mg, 0.0))))
        ma = self._sa / n
        moving = std > self.cfg.calib_max_gyro_std or np.linalg.norm(ma) < 1e-6
        if moving and self._tries < 3:
            self._tries += 1
            self._cn, self._cel = 0, 0.0
            self._sg[:] = 0.0
            self._sg2[:] = 0.0
            self._sa[:] = 0.0
            self.msg = "Motion during calibration - restarting (hold still)"
            return
        self.bg = mg.copy()
        if not self._bias_only:
            self.q = quat_from_accel(ma)
            self._q_prev = self.q.copy()
            self.P = np.diag([self.cfg.init_att_std ** 2] * 3 + [self.cfg.init_bias_std ** 2] * 3)
        else:
            self.P[3:, 3:] = np.eye(3) * self.cfg.init_bias_std ** 2
            self.P[:3, 3:] = 0.0
            self.P[3:, :3] = 0.0
        self.state = self.RUN
        self.msg = "Running" + (" (calibrated with motion!)" if moving else "")

    # -- prediction --------------------------------------------------------------------------
    def _Q(self, dt):
        sg2 = self.cfg.gyro_noise ** 2
        sb2 = self.cfg.gyro_bias_rw ** 2
        Q = np.zeros((6, 6))
        i = np.arange(3)
        Q[i, i] = sg2 * dt + sb2 * dt ** 3 / 3.0
        Q[i, i + 3] = Q[i + 3, i] = -sb2 * dt ** 2 / 2.0
        Q[i + 3, i + 3] = sb2 * dt
        return Q

    def _predict(self, gyro, dt):
        w = gyro - self.bg
        th = w * dt
        self.q = quat_normalize(quat_mul(self.q, quat_from_rotvec(th)))
        F = np.eye(6)
        F[:3, :3] = rotvec_to_rot(th).T            # exp(-[w dt]x)
        F[:3, 3:] = -np.eye(3) * dt
        self.P = F @ self.P @ F.T + self._Q(dt)
        self._sanitize_cov()

    # -- accelerometer update ----------------------------------------------------------------
    def _update_acc(self, acc):
        c = self.cfg
        n = float(np.linalg.norm(acc))
        if not math.isfinite(n) or n < 1e-6:
            self.msg = "Accel invalid (zero) - update skipped"
            return
        dev = abs(n - c.gravity) / c.gravity
        if dev > c.accel_reject_tol:
            self.rejected += 1
            self.msg = "Accel rejected (|a| off by %.0f%%)" % (dev * 100)
            return
        Rm = quat_to_rot(self.q)
        h = -Rm[2, :]                              # expected unit specific force in body axes
        z = acc / n
        y = z - h
        Sk = skew(h)
        H = np.zeros((3, 6))
        H[:, :3] = Sk
        Rn = np.eye(3) * (c.accel_noise ** 2) * (1.0 + (dev / c.accel_soft_tol) ** 2)
        PHt = self.P @ H.T
        S = H @ PHt + Rn
        try:
            K = np.linalg.solve(S, PHt.T).T
        except np.linalg.LinAlgError:
            self.msg = "Accel update skipped (singular S)"
            return
        dx = K @ y
        dth, dbg = dx[:3], dx[3:]
        if not np.all(np.isfinite(dx)) or np.linalg.norm(dth) > 0.8:
            self.rejected += 1
            self.msg = "Accel update rejected (innovation too large)"
            return
        IKH = np.eye(6) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ Rn @ K.T           # Joseph form
        self.q = quat_normalize(quat_mul(self.q, quat_from_rotvec(dth)))
        self.bg = self.bg + dbg
        G = np.eye(6)
        G[:3, :3] = np.eye(3) - 0.5 * skew(dth)                # error-state reset
        self.P = G @ self.P @ G.T
        self._sanitize_cov()
        self.updates += 1
        self.msg = "Running" if dev < c.accel_soft_tol else "Running (accel down-weighted)"

    # -- numerical guards --------------------------------------------------------------------
    def _sanitize_cov(self):
        P = self.P
        if not np.all(np.isfinite(P)):
            self._recover("covariance not finite")
            return
        P = 0.5 * (P + P.T)
        d = np.diag(P).copy()
        if np.any(d < 0):
            self._recover("covariance not PSD")
            return
        np.fill_diagonal(P, np.clip(d, 1e-12, self.cfg.max_cov))
        self.P = P

    def _finish(self):
        if not (np.all(np.isfinite(self.q)) and np.all(np.isfinite(self.bg))):
            self._recover("state not finite")
            return
        self.q = quat_normalize(self.q)
        if float(self._q_prev @ self.q) < 0.0:                 # sign continuity
            self.q = -self.q
        self._q_prev = self.q.copy()

    def _recover(self, why):
        self.recoveries += 1
        if not np.all(np.isfinite(self.q)):
            self.q = _IDENT_Q.copy()
        if not np.all(np.isfinite(self.bg)):
            self.bg = np.zeros(3)
        self.P = np.diag([0.1 ** 2] * 3 + [self.cfg.init_bias_std ** 2] * 3)
        self.msg = "Numerical reset: " + why

    @property
    def euler_deg(self):
        return np.degrees(quat_to_euler(self.q))


# ----------------------------------------------------------------------------------------------
# Motor mixer (hover)
# ----------------------------------------------------------------------------------------------
# Quad-X layout in the FRD body frame (PX4 numbering):  (name, position, x sign, y sign, spin)
#   spin +1 = propeller turns CCW seen from above -> its reaction torque on the airframe is
#   clockwise from above = POSITIVE yaw (z down).  Thrust points up = -z_body.
#        front
#     M3(CW)  M1(CCW)
#     M2(CCW) M4(CW)
#        rear
MOTORS = (("M1", "FR", +1, +1, +1), ("M2", "RL", -1, -1, +1),
          ("M3", "FL", +1, -1, -1), ("M4", "RR", -1, +1, -1))

# Mixing matrix columns = [roll, pitch, yaw] torque commands -> per-motor thrust delta.
#   roll  (+ = right wing down): extra thrust on LEFT motors   -> -y_sign
#   pitch (+ = nose up):         extra thrust on FRONT motors  -> +x_sign
#   yaw   (+ = nose right):      extra thrust on CCW motors    -> +spin
MIX_MATRIX = np.array([[-m[3], m[2], m[4]] for m in MOTORS], float)


@dataclass
class MixerConfig:
    twr: float = 2.0              # max total thrust / weight (sets hover thrust = 1/TWR per motor)
    max_rpm: float = 12000.0      # for the RPM read-out only
    idle_thrust: float = 0.03     # minimum thrust fraction per motor (spin-up floor)
    kp_rp: float = 0.30           # thrust fraction per rad            (roll/pitch)
    ki_rp: float = 0.03
    kd_rp: float = 0.02           # thrust fraction per rad/s
    kp_yaw: float = 0.10
    ki_yaw: float = 0.01
    kd_yaw: float = 0.05
    i_limit: float = 0.10         # integrator clamp (thrust fraction)
    use_integral: bool = False    # OFF for bench tests (nothing can correct the error); ON for flight
    min_cos_tilt: float = 0.5     # tilt compensation limit
    max_tilt_deg: float = 60.0    # beyond this the mixer outputs idle (fail-safe)


class MixerOutput:
    __slots__ = ("thrust", "speed", "rpm", "pwm", "collective", "torque", "saturated", "status", "active")

    def __init__(self):
        self.thrust = np.zeros(4)      # per-motor thrust fraction of max thrust (0..1)
        self.speed = np.zeros(4)       # per-motor speed fraction (thrust ~ speed^2) -> sqrt(thrust)
        self.rpm = np.zeros(4)
        self.pwm = np.full(4, 1000.0)  # 1000..2000 us equivalent
        self.collective = 0.0
        self.torque = np.zeros(3)      # roll, pitch, yaw commands (thrust-fraction units)
        self.saturated = False
        self.status = "Disabled"
        self.active = False


class MotorMixer:
    """Hover controller + mixer: holds level attitude and the captured yaw, outputs 4 motor speeds.

    Thrust domain: hover thrust per motor f_h = 1/TWR, collective f_c = f_h / max(cos(tilt), min_cos)
    (tilt compensation keeps vertical thrust = weight). PD(+I) attitude control on the MEKF quaternion
    error produces roll/pitch/yaw commands, mixed with MIX_MATRIX. Saturation is resolved by first
    shifting the collective, then scaling the attitude commands (attitude has priority over thrust).
    Motor speed = sqrt(thrust fraction) because thrust ~ omega^2.
    Output only; nothing is transmitted to the flight controller."""

    def __init__(self, cfg=None):
        self.cfg = cfg or MixerConfig()
        self.yaw_sp = 0.0
        self.reset()

    def reset(self):
        self._I = np.zeros(3)
        self.out = MixerOutput()

    @property
    def hover_thrust(self):
        return 1.0 / max(self.cfg.twr, 1.0)

    def set_yaw_hold(self, yaw_rad):
        self.yaw_sp = float(yaw_rad)

    def update(self, q, omega, dt, enabled=True, filter_ready=True):
        c = self.cfg
        o = MixerOutput()
        self.out = o
        if not enabled:
            o.status = "Disabled"
            return o
        if not filter_ready:
            o.status = "Waiting for attitude filter"
            return o
        R = quat_to_rot(q)
        cos_t = float(R[2, 2])                           # cos(tilt) of body z vs world down
        f_min = c.idle_thrust
        if cos_t < math.cos(math.radians(c.max_tilt_deg)):
            o.thrust[:] = f_min
            o.status = "TILT LIMIT - idle"
            self._finish(o)
            return o
        q_des = euler_to_quat(0.0, 0.0, self.yaw_sp)     # level, hold yaw
        qe = quat_mul(np.array([q[0], -q[1], -q[2], -q[3]]), q_des)   # q^-1 (x) q_des = body-frame error
        e = 2.0 * qe[1:] * (1.0 if qe[0] >= 0.0 else -1.0)
        kp = np.array([c.kp_rp, c.kp_rp, c.kp_yaw])
        ki = np.array([c.ki_rp, c.ki_rp, c.ki_yaw])
        kd = np.array([c.kd_rp, c.kd_rp, c.kd_yaw])
        if not c.use_integral:
            self._I[:] = 0.0
        u = kp * e + ki * self._I - kd * omega
        d = MIX_MATRIX @ u
        base = self.hover_thrust / max(cos_t, c.min_cos_tilt)
        f_max = 1.0
        span = float(d.max() - d.min())
        sat = False
        if span > f_max - f_min:                          # not enough range: scale attitude commands
            k = (f_max - f_min) / span
            d = d * k
            u = u * k
            sat = True
        lo, hi = f_min - float(d.min()), f_max - float(d.max())
        if base < lo or base > hi:                        # shift collective to fit
            base = min(max(base, lo), hi)
            sat = True
        f = np.clip(base + d, f_min, f_max)
        if not sat:                                       # integrate only when not saturated and near level
            lim = c.i_limit / np.maximum(ki, 1e-9)
            self._I = np.clip(self._I + np.where(np.abs(e) < 0.15, e, 0.0) * dt, -lim, lim)
        o.thrust = f
        o.collective = base
        o.torque = u
        o.saturated = sat
        o.status = "SATURATED" if sat else "Hovering"
        self._finish(o)
        return o

    def _finish(self, o):
        o.active = True
        o.speed = np.sqrt(np.clip(o.thrust, 0.0, 1.0))
        o.rpm = o.speed * self.cfg.max_rpm
        o.pwm = 1000.0 + 1000.0 * o.speed


# ----------------------------------------------------------------------------------------------
# 3D drone view
# ----------------------------------------------------------------------------------------------
def _tf(M):
    return pg.Transform3D(*[float(v) for v in np.asarray(M, dtype=float).ravel()])


def _translation(p):
    M = np.eye(4)
    M[:3, 3] = p
    return M


class Drone3D:
    """Reusable OpenGL scene. Only item transforms change at runtime."""

    ARM = 0.35            # motor offset along x and y [view units]

    def __init__(self):
        if not HAVE_GL:
            raise RuntimeError("PyQtGraph OpenGL unavailable: " + GL_IMPORT_ERROR)
        self.widget = gl.GLViewWidget()
        self.widget.setBackgroundColor((18, 20, 24))
        self.widget.setCameraPosition(distance=2.6, elevation=24, azimuth=-150)
        self.widget.setMinimumSize(480, 320)
        self._parts = []               # (item, local 4x4)
        self._build_world()
        self._build_drone()
        self.set_orientation(_IDENT_Q)

    def _add_static(self, item):
        self.widget.addItem(item)

    def _add_part(self, item, local=None):
        self.widget.addItem(item)
        self._parts.append((item, np.eye(4) if local is None else local))

    def _build_world(self):
        grid = gl.GLGridItem()
        grid.setSize(4, 4)
        grid.setSpacing(0.25, 0.25)
        grid.setColor((90, 95, 105, 120))
        grid.translate(0, 0, -0.5)
        self._add_static(grid)
        # Stationary world axes. NED -> view: N=+x, E=-y(view), D=-z(view). X red, Y green, Z blue.
        L = 1.1
        ends = [NED_TO_DISPLAY @ np.array(v) * L for v in ([1, 0, 0], [0, 1, 0], [0, 0, 1])]
        pos, col = [], []
        for e, c in zip(ends, [(1, 0.25, 0.25, 0.55), (0.25, 1, 0.25, 0.55), (0.35, 0.5, 1, 0.55)]):
            pos += [[0, 0, 0], list(e)]
            col += [c, c]
        self._add_static(gl.GLLinePlotItem(pos=np.array(pos), color=np.array(col), width=2,
                                           mode="lines", antialias=True))
        try:
            for txt, e in zip(("N", "E", "D"), ends):
                t = gl.GLTextItem(pos=e * 1.05, text=txt, color=(200, 200, 210, 255))
                self._add_static(t)
        except Exception:
            pass                        # GLTextItem not present in older PyQtGraph: skip labels

    def _build_drone(self):
        a = self.ARM
        # Body (FRD coordinates: -z is "up")
        hx, hy, hz = 0.13, 0.085, 0.035
        V = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
        idx = lambda sx, sy, sz: ((sx > 0) * 4 + (sy > 0) * 2 + (sz > 0))
        quads = {   # name: 4 corner indices (ccw from outside)
            "top":    [idx(-1, -1, -1), idx(1, -1, -1), idx(1, 1, -1), idx(-1, 1, -1)],
            "bottom": [idx(-1, -1, 1), idx(-1, 1, 1), idx(1, 1, 1), idx(1, -1, 1)],
            "front":  [idx(1, -1, -1), idx(1, -1, 1), idx(1, 1, 1), idx(1, 1, -1)],
            "back":   [idx(-1, -1, -1), idx(-1, 1, -1), idx(-1, 1, 1), idx(-1, -1, 1)],
            "right":  [idx(-1, 1, -1), idx(1, 1, -1), idx(1, 1, 1), idx(-1, 1, 1)],
            "left":   [idx(-1, -1, -1), idx(-1, -1, 1), idx(1, -1, 1), idx(1, -1, -1)],
        }
        cols = {"top": (0.25, 0.55, 0.95, 1), "bottom": (0.18, 0.18, 0.2, 1), "front": (0.95, 0.85, 0.2, 1),
                "back": (0.3, 0.3, 0.33, 1), "right": (0.35, 0.35, 0.4, 1), "left": (0.35, 0.35, 0.4, 1)}
        faces, fcol = [], []
        for name, q in quads.items():
            faces += [[q[0], q[1], q[2]], [q[0], q[2], q[3]]]
            fcol += [cols[name]] * 2
        md = gl.MeshData(vertexes=V, faces=np.array(faces), faceColors=np.array(fcol))
        self._add_part(gl.GLMeshItem(meshdata=md, smooth=False, shader="shaded", drawEdges=True,
                                     edgeColor=(0, 0, 0, 1)))
        # Arms in X configuration
        motors = [(a, a), (a, -a), (-a, -a), (-a, a)]          # FR, FL, RL, RR (x fwd, y right)
        pos = []
        for mx, my in motors:
            pos += [[0, 0, 0], [mx, my, 0]]
        self._add_part(gl.GLLinePlotItem(pos=np.array(pos, float), color=(0.75, 0.75, 0.8, 1), width=6,
                                         mode="lines", antialias=True))
        # Motors (front = green, rear = orange) and propeller rings
        th = np.linspace(0, 2 * np.pi, 33)
        ring = np.stack([0.17 * np.cos(th), 0.17 * np.sin(th), np.full_like(th, -0.045)], axis=1)
        for mx, my in motors:
            front = mx > 0
            col = (0.2, 0.9, 0.35, 1) if front else (1.0, 0.55, 0.15, 1)
            sph = gl.MeshData.sphere(rows=8, cols=12, radius=0.05)
            item = gl.GLMeshItem(meshdata=sph, smooth=True, color=col, shader="shaded")
            self._add_part(item, _translation([mx, my, -0.03]))
            rg = gl.GLLinePlotItem(pos=ring, color=col, width=2, mode="line_strip", antialias=True)
            self._add_part(rg, _translation([mx, my, 0]))
        # Front direction indicator (arrow along +x)
        tip = 0.62
        arrow = np.array([[0.13, 0, -0.04], [tip, 0, -0.04],
                          [tip, 0, -0.04], [tip - 0.08, 0.04, -0.04],
                          [tip, 0, -0.04], [tip - 0.08, -0.04, -0.04]], float)
        self._add_part(gl.GLLinePlotItem(pos=arrow, color=(1, 0.9, 0.1, 1), width=4, mode="lines",
                                         antialias=True))
        # Body axes (x red, y green, z blue), z points DOWN in FRD
        L = 0.45
        pos = np.array([[0, 0, 0], [L, 0, 0], [0, 0, 0], [0, L, 0], [0, 0, 0], [0, 0, L]], float)
        col = np.array([(1, 0.2, 0.2, 1)] * 2 + [(0.2, 1, 0.2, 1)] * 2 + [(0.3, 0.5, 1, 1)] * 2)
        self._add_part(gl.GLLinePlotItem(pos=pos, color=col, width=3, mode="lines", antialias=True))

    def set_orientation(self, q):
        M = np.eye(4)
        M[:3, :3] = NED_TO_DISPLAY @ quat_to_rot(q)
        for item, local in self._parts:
            item.setTransform(_tf(M @ local))


# ----------------------------------------------------------------------------------------------
# Quaternion plot
# ----------------------------------------------------------------------------------------------
class TelemetryPlot:
    NAMES = ("qw", "qx", "qy", "qz")
    COLORS = ((235, 235, 235), (255, 90, 90), (90, 220, 110), (90, 160, 255))

    def __init__(self, window_s=HISTORY_SECONDS, max_rate=MAX_HISTORY_RATE_HZ):
        self.window_s = window_s
        self.cap = int(window_s * max_rate * 1.2)
        self._t = np.zeros(self.cap)
        self._d = np.zeros((4, self.cap))
        self._i = 0
        self._n = 0
        self.widget = pg.PlotWidget()
        self.widget.setBackground((18, 20, 24))
        pi = self.widget.getPlotItem()
        pi.showGrid(x=True, y=True, alpha=0.25)
        pi.setLabel("bottom", "time", units="s")
        pi.setLabel("left", "quaternion")
        pi.addLegend(offset=(10, 5))
        pi.setYRange(-1.1, 1.1, padding=0)
        pi.setXRange(-window_s, 0, padding=0)
        pi.setMouseEnabled(x=False, y=False)
        pi.hideButtons()
        self.curves = [pi.plot(pen=pg.mkPen(c, width=1.5), name=n) for n, c in zip(self.NAMES, self.COLORS)]
        self.widget.setMinimumHeight(150)

    def push(self, t, q):
        i = self._i
        self._t[i] = t
        self._d[:, i] = q
        self._i = (i + 1) % self.cap
        if self._n < self.cap:
            self._n += 1

    def clear(self):
        self._i = 0
        self._n = 0

    def refresh(self):
        n = self._n
        if n < 2:
            return
        if n < self.cap:
            t, d = self._t[:n], self._d[:, :n]
        else:
            order = np.r_[self._i:self.cap, 0:self._i]
            t, d = self._t[order], self._d[:, order]
        x = t - t[-1]
        k = int(np.searchsorted(x, -self.window_s))
        x, d = x[k:], d[:, k:]
        step = max(1, len(x) // 1500)           # decimate for drawing only
        x = x[::step]
        for j, c in enumerate(self.curves):
            c.setData(x, d[j, ::step], skipFiniteCheck=True)


class MixerPanel:
    """Dockable UI for the motor mixer: settings, top-view motor read-outs, status."""

    def __init__(self, mixer, get_yaw_rad, on_reset):
        self.mixer, self.get_yaw = mixer, get_yaw_rad
        mono = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)
        mono.setPointSize(9)
        self.dock = QtWidgets.QDockWidget("Motor mixer - hover")
        self.dock.setFeatures(QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetMovable |
                              QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetFloatable)
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(4)
        self.chk = QtWidgets.QCheckBox("Enable mixer (hover)")
        self.chk.setChecked(True)
        v.addWidget(self.chk)
        note = QtWidgets.QLabel("Computed outputs only - nothing is sent to the flight controller.")
        note.setWordWrap(True)
        note.setStyleSheet("color:#9aa0a6;")
        v.addWidget(note)

        # motor read-outs in top view (front at top)
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(6)
        self.bars, self.txt = [], []
        for i, (name, pos, xs, ys, spin) in enumerate(MOTORS):
            cell = QtWidgets.QVBoxLayout()
            head = QtWidgets.QLabel("%s %s  %s" % (name, pos, "CCW" if spin > 0 else "CW"))
            head.setStyleSheet("font-weight:bold;")
            bar = QtWidgets.QProgressBar()
            bar.setRange(0, 1000)
            bar.setTextVisible(False)
            bar.setFixedHeight(14)
            t = QtWidgets.QLabel("--")
            t.setFont(mono)
            for wdg in (head, bar, t):
                cell.addWidget(wdg)
            grid.addLayout(cell, 0 if xs > 0 else 1, 1 if ys > 0 else 0)
            self.bars.append(bar)
            self.txt.append(t)
        self._bar_sat = None
        self._style_bars(False)
        v.addWidget(QtWidgets.QLabel("Top view  (front = up)"))
        v.addLayout(grid)
        self.lbl_info = QtWidgets.QLabel("--")
        self.lbl_info.setFont(mono)
        self.lbl_info.setWordWrap(True)
        v.addWidget(self.lbl_info)
        self.lbl_status = QtWidgets.QLabel("--")
        self.lbl_status.setStyleSheet("font-weight:bold;")
        v.addWidget(self.lbl_status)

        # settings
        c = mixer.cfg
        f = QtWidgets.QGridLayout()
        f.setSpacing(3)
        r = 0
        for label, name, lo, hi, dec, step in (
                ("Thrust/weight (TWR)", "twr", 1.1, 20.0, 2, 0.1),
                ("Max RPM", "max_rpm", 100.0, 100000.0, 0, 500.0),
                ("Idle thrust frac", "idle_thrust", 0.0, 0.3, 3, 0.01)):
            f.addWidget(QtWidgets.QLabel(label), r, 0)
            f.addWidget(self._spin(c, name, lo, hi, dec, step), r, 1, 1, 3)
            r += 1
        self.lbl_hover = QtWidgets.QLabel("--")
        f.addWidget(QtWidgets.QLabel("Hover thrust/speed"), r, 0)
        f.addWidget(self.lbl_hover, r, 1, 1, 3)
        r += 1
        for j, h in enumerate(("Kp", "Ki", "Kd")):
            f.addWidget(QtWidgets.QLabel(h), r, 1 + j)
        r += 1
        for label, names in (("Roll/Pitch", ("kp_rp", "ki_rp", "kd_rp")), ("Yaw", ("kp_yaw", "ki_yaw", "kd_yaw"))):
            f.addWidget(QtWidgets.QLabel(label), r, 0)
            for j, n in enumerate(names):
                f.addWidget(self._spin(c, n, 0.0, 20.0, 3, 0.01), r, 1 + j)
            r += 1
        self.chk_i = QtWidgets.QCheckBox("Use integral term (flight; off for bench)")
        self.chk_i.setChecked(c.use_integral)
        self.chk_i.toggled.connect(lambda on: setattr(c, "use_integral", bool(on)))
        f.addWidget(self.chk_i, r, 0, 1, 4)
        v.addLayout(f)
        hb = QtWidgets.QHBoxLayout()
        b1 = QtWidgets.QPushButton("Hold current yaw")
        b1.clicked.connect(lambda *_: self.mixer.set_yaw_hold(self.get_yaw()))
        b2 = QtWidgets.QPushButton("Reset integrators")
        b2.clicked.connect(lambda *_: on_reset())
        hb.addWidget(b1)
        hb.addWidget(b2)
        v.addLayout(hb)
        v.addStretch(1)
        self.dock.setWidget(w)
        w.setMinimumWidth(300)

    def _spin(self, cfg, name, lo, hi, dec, step):
        s = QtWidgets.QDoubleSpinBox()
        s.setDecimals(dec)
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setValue(getattr(cfg, name))
        s.valueChanged.connect(lambda val, n=name: setattr(cfg, n, float(val)))
        return s

    def _style_bars(self, sat):
        if sat == self._bar_sat:
            return
        self._bar_sat = sat
        col = "#ff9f40" if sat else "#4cd964"
        for b in self.bars:
            b.setStyleSheet("QProgressBar{border:1px solid #555;background:#16181c;}"
                            "QProgressBar::chunk{background:%s;}" % col)

    def update(self, o):
        self._style_bars(o.saturated)
        for i in range(4):
            self.bars[i].setValue(int(round(float(o.speed[i]) * 1000)))
            self.txt[i].setText("%3.0f%% %5.0frpm\n%4.0fus  T%.2f" % (
                o.speed[i] * 100, o.rpm[i], o.pwm[i], o.thrust[i]))
        self.lbl_info.setText("collective %.3f   cmd R%+.3f P%+.3f Y%+.3f" % (o.collective, *o.torque))
        col = {"Hovering": "#4cd964", "SATURATED": "#ff9f40"}.get(o.status, "#9aa0a6")
        if o.status.startswith("TILT"):
            col = "#ff4d4d"
        self.lbl_status.setText(o.status)
        self.lbl_status.setStyleSheet("font-weight:bold;color:%s;" % col)
        h = self.mixer.hover_thrust
        self.lbl_hover.setText("thrust %.3f  speed %.1f%%" % (h, 100 * math.sqrt(h)))

    @property
    def enabled(self):
        return self.chk.isChecked()


# ----------------------------------------------------------------------------------------------
# Firmware panel (calibrated values, firmware quaternion, stored bias, parameter editor)
# ----------------------------------------------------------------------------------------------
class ParamRow:
    """One firmware parameter: editable value + stored value. A local edit is never overwritten by
    incoming data until it has been sent (and confirmed by the firmware) or the user reloads."""

    def __init__(self, link, name, decimals=5, step=0.001, is_bool=False, with_button=False):
        self.link, self.name, self.is_bool, self.decimals = link, name, is_bool, decimals
        self.edited = False
        self._loading = False
        self._rev_seen = -1
        self._range = None
        if is_bool:
            self.w = QtWidgets.QCheckBox()
            self.w.toggled.connect(self._on_edit)
        else:
            self.w = QtWidgets.QDoubleSpinBox()
            self.w.setDecimals(decimals)
            self.w.setSingleStep(step)
            self.w.setRange(-1e6, 1e6)
            self.w.setKeyboardTracking(False)
            self.w.valueChanged.connect(self._on_edit)
        self.w.setEnabled(False)
        self.stored = QtWidgets.QLabel("--")
        self.btn = None
        if with_button:
            self.btn = QtWidgets.QPushButton("Send")
            self.btn.setFixedWidth(46)
            self.btn.clicked.connect(lambda *_: self.send())

    def value(self):
        return bool(self.w.isChecked()) if self.is_bool else float(self.w.value())

    def set_local(self, value):
        """Programmatic change of the editor (does not count as a user edit)."""
        self._loading = True
        try:
            if self.is_bool:
                self.w.setChecked(bool(value))
            else:
                self.w.setValue(float(value))
        finally:
            self._loading = False

    def mark_edited(self):
        self.edited = True
        self._style()

    def _on_edit(self, *_):
        if self._loading:
            return
        self.mark_edited()

    def _style(self):
        self.w.setStyleSheet("background:#5a4a1e;" if self.edited else "")

    def send(self):
        ok = self.link.set_param(self.name, self.value())
        if ok:
            p = self.link.params.get(self.name)
            self._rev_seen = p.rev if p is not None else -1
        return ok

    def discard(self):
        self.edited = False
        self._rev_seen = -1
        self._style()

    def refresh(self):
        p = self.link.params.get(self.name)
        if p is None:
            self.w.setEnabled(False)
            self.stored.setText("--")
            return
        self.w.setEnabled(True)
        if not self.is_bool and math.isfinite(p.lo) and math.isfinite(p.hi):
            if self._range != (p.lo, p.hi):          # QDoubleSpinBox rounds its limits, so cache what we set
                self._range = (p.lo, p.hi)
                self._loading = True
                try:
                    self.w.setRange(p.lo, p.hi)
                finally:
                    self._loading = False
        self.stored.setText(("%d" % int(p.value)) if self.is_bool else ("%.*f" % (self.decimals, p.value)))
        if self.edited:
            same = (bool(self.w.isChecked()) == bool(p.value)) if self.is_bool \
                else abs(self.w.value() - p.value) <= 10.0 ** (-self.decimals)
            if same and p.rev != self._rev_seen:       # firmware now holds exactly what the editor shows
                self.edited = False
                self._style()
        if not self.edited and (self._rev_seen != p.rev or self._rev_seen < 0):
            self.set_local(p.value)
            self._rev_seen = p.rev
            self._style()


class FirmwarePanel:
    """Dock with the firmware's view of the world and the interface to change its stored values."""

    # (title, parameter name pattern, decimals, step)
    BIAS_GROUPS = (
        ("Gyro bias  [deg/s]  SENSOR axes  (cal.gyro_bias_*)", "cal.gyro_bias_%s", 5, 0.001),
        ("Accel offset  [g]  SENSOR axes  (cal.accel_offset_*)", "cal.accel_offset_%s", 5, 0.0005),
        ("Accel scale  SENSOR axes  (cal.accel_scale_*)", "cal.accel_scale_%s", 5, 0.0005),
        ("Residual gyro bias seed [deg/s]  BODY axes FRD  (mekf.bias_*)", "mekf.bias_%s", 5, 0.001),
    )

    def __init__(self, link, get_dash_bias_dps, confirm=None):
        self.link = link
        self.get_dash_bias = get_dash_bias_dps
        self.confirm = confirm or self._confirm_dialog
        self._log_pos = 0
        self._all_names = None
        mono = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)
        mono.setPointSize(9)
        self.mono = mono
        self.dock = QtWidgets.QDockWidget("Firmware - calibration, bias, quaternion")
        self.dock.setFeatures(QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetMovable |
                              QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetFloatable)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._build_live_tab(), "Live / bias")
        self.tabs.addTab(self._build_all_tab(), "All parameters")
        self.tabs.addTab(self._build_log_tab(), "Log")
        self.dock.setWidget(self.tabs)
        self.tabs.setMinimumWidth(360)

    # -- helpers ------------------------------------------------------------------------------
    def _confirm_dialog(self, text):
        B = QtWidgets.QMessageBox.StandardButton
        r = QtWidgets.QMessageBox.question(self.dock, "Confirm", text, B.Yes | B.No, B.No)
        return r == B.Yes

    def _mk(self, text="--", wrap=False):
        lab = QtWidgets.QLabel(text)
        lab.setFont(self.mono)
        lab.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        if wrap:
            lab.setWordWrap(True)
        return lab

    def _btn(self, text, slot):
        b = QtWidgets.QPushButton(text)
        b.clicked.connect(lambda *_: slot())
        return b

    # -- tab 1: live values + bias editor --------------------------------------------------------
    def _build_live_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(4)
        self.lbl = {}
        g = QtWidgets.QGridLayout()
        g.setSpacing(2)
        rows = (("link", "FC link"), ("store", "Stored config"), ("prompt", "Calibration"),
                ("acc", "Accel [g]  (calibrated)"), ("gyr", "Gyro [dps] (calibrated)"),
                ("fstate", "FC filter"), ("fq", "FC quaternion"), ("fe", "FC roll/pitch/yaw"),
                ("dq", "Dashboard quat"), ("delta", "FC vs dashboard"),
                ("fbias", "Live bias FC (body)"), ("dbias", "Live bias dash (body)"),
                ("mix", "FC mixer"))
        for r, (k, name) in enumerate(rows):
            g.addWidget(QtWidgets.QLabel(name), r, 0)
            self.lbl[k] = self._mk(wrap=(k in ("store", "prompt", "link")))
            g.addWidget(self.lbl[k], r, 1)
        g.setColumnStretch(1, 1)
        v.addLayout(g)

        # FC motor outputs (top view, front = up)
        mg = QtWidgets.QGridLayout()
        mg.setSpacing(4)
        self.bars, self.mtxt = [], []
        for i, (name, pos, xs, ys, spin) in enumerate(MOTORS):
            cell = QtWidgets.QVBoxLayout()
            head = QtWidgets.QLabel("%s %s %s" % (name, pos, "CCW" if spin > 0 else "CW"))
            bar = QtWidgets.QProgressBar()
            bar.setRange(0, 1000)
            bar.setTextVisible(False)
            bar.setFixedHeight(12)
            t = self._mk()
            for wdg in (head, bar, t):
                cell.addWidget(wdg)
            mg.addLayout(cell, 0 if xs > 0 else 1, 1 if ys > 0 else 0)
            self.bars.append(bar)
            self.mtxt.append(t)
        v.addWidget(QtWidgets.QLabel("Firmware motor outputs (top view, front = up)"))
        v.addLayout(mg)

        # bias / calibration editor
        self.rows = []
        self.group_rows = []
        for title, pat, dec, step in self.BIAS_GROUPS:
            box = QtWidgets.QGroupBox(title)
            bg = QtWidgets.QGridLayout(box)
            bg.setSpacing(3)
            grow = []
            for j, ax in enumerate("xyz"):
                bg.addWidget(QtWidgets.QLabel(ax.upper()), 0, j, QtCore.Qt.AlignmentFlag.AlignHCenter)
                pr = ParamRow(self.link, pat % ax, dec, step)
                bg.addWidget(pr.w, 1, j)
                bg.addWidget(pr.stored, 2, j, QtCore.Qt.AlignmentFlag.AlignHCenter)
                grow.append(pr)
                self.rows.append(pr)
            sb = self._btn("Send", lambda rs=grow: self._send_rows(rs))
            bg.addWidget(sb, 1, 3)
            bg.addWidget(QtWidgets.QLabel("stored"), 2, 3)
            self.group_rows.append(grow)
            v.addWidget(box)

        b1 = QtWidgets.QHBoxLayout()
        b1.addWidget(self._btn("Send all changed", self.send_all_changed))
        b1.addWidget(self._btn("Discard edits / reload", self.reload_from_fc))
        v.addLayout(b1)
        b2 = QtWidgets.QHBoxLayout()
        b2.addWidget(self._btn("Dashboard bias -> seed", self.copy_dashboard_bias))
        self.btn_save = self._btn("Save to flash", self.link.save)
        b2.addWidget(self.btn_save)
        v.addLayout(b2)
        b3 = QtWidgets.QHBoxLayout()
        b3.addWidget(self._btn("Reload stored (LOAD)", self.load_stored))
        b3.addWidget(self._btn("CAL BIAS (keep still)", self.cal_bias))
        v.addLayout(b3)
        b4 = QtWidgets.QHBoxLayout()
        b4.addWidget(self._btn("CAL FULL...", self.cal_full))
        b4.addWidget(self._btn("Clear stored...", self.clear_stored))
        b4.addWidget(self._btn("Reboot FC", self.reboot))
        v.addLayout(b4)
        note = QtWidgets.QLabel("'Send' changes the value live in the FC (not stored yet). "
                                "'Save to flash' stores everything. cal.* values are in the SENSOR frame; "
                                "mekf.bias_* and the live bias read-outs are in the BODY (FRD) frame "
                                "(sensor frame mapped by the FC's SENSOR_TO_BODY).")
        note.setWordWrap(True)
        note.setStyleSheet("color:#9aa0a6;")
        v.addWidget(note)
        v.addStretch(1)
        return w

    # -- tab 2: every parameter ---------------------------------------------------------------
    def _build_all_tab(self):
        self.all_scroll = QtWidgets.QScrollArea()
        self.all_scroll.setWidgetResizable(True)
        self.all_rows = []
        holder = QtWidgets.QLabel("Parameters appear after the firmware answers LIST.")
        holder.setWordWrap(True)
        self.all_scroll.setWidget(holder)
        return self.all_scroll

    def _rebuild_all(self):
        names = tuple(self.link.params.keys())
        if names == self._all_names:
            return
        self._all_names = names
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setSpacing(2)
        for j, h in enumerate(("name", "value", "stored", "")):
            g.addWidget(QtWidgets.QLabel(h), 0, j)
        self.all_rows = []
        prefix = None
        r = 1
        for name, p in self.link.params.items():
            pre = name.split(".")[0]
            if pre != prefix:
                prefix = pre
                head = QtWidgets.QLabel("<b>%s.*</b>" % pre)
                g.addWidget(head, r, 0, 1, 4)
                r += 1
            dec = 9 if abs(p.hi) <= 0.2 else (6 if abs(p.hi) <= 1.5 else (4 if abs(p.hi) <= 200 else 1))
            pr = ParamRow(self.link, name, dec, 10.0 ** (-min(dec, 3)), is_bool=p.is_bool, with_button=True)
            g.addWidget(QtWidgets.QLabel(name), r, 0)
            g.addWidget(pr.w, r, 1)
            g.addWidget(pr.stored, r, 2)
            g.addWidget(pr.btn, r, 3)
            self.all_rows.append(pr)
            r += 1
        g.setRowStretch(r, 1)
        self.all_scroll.setWidget(w)

    # -- tab 3: log ---------------------------------------------------------------------------
    def _build_log_tab(self):
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(500)
        self.log_view.setFont(self.mono)
        return self.log_view

    # -- actions ------------------------------------------------------------------------------
    def _send_rows(self, rows):
        for r in rows:
            if r.edited:
                r.send()

    def send_all_changed(self):
        self._send_rows(self.rows + self.all_rows)

    def reload_from_fc(self):
        for r in self.rows + self.all_rows:
            r.discard()
        self.link.request_sync()

    def load_stored(self):
        for r in self.rows + self.all_rows:
            r.discard()
        self.link.load()

    def copy_dashboard_bias(self):
        """Put the dashboard MEKF's gyro-bias estimate into the residual-bias seed editors (not sent)."""
        b = self.get_dash_bias()
        for pr, val in zip(self.group_rows[3], b):
            pr.set_local(float(val))
            pr.mark_edited()

    def cal_bias(self):
        self.link.cal_bias()

    def cal_full(self):
        if self.confirm("Start the FULL calibration on the flight controller?\n\n"
                        "You will have to place the board on all six faces and press the button for each."):
            self.link.cal_full()

    def clear_stored(self):
        if self.confirm("Erase the stored configuration in the flight controller?\n\n"
                        "The next boot will run the full calibration."):
            self.link.clear()

    def reboot(self):
        if self.confirm("Reboot the flight controller?"):
            self.link.reboot()

    # -- periodic update ------------------------------------------------------------------------
    def update(self, d):
        """d: dict prepared by the dashboard (sample, fc, ages, dashboard attitude, ...)."""
        L = self.lbl
        lk = self.link
        fc, s = d["fc"], d["sample"]

        if d["fc_fresh"]:
            L["link"].setText("FC telemetry OK  |  cmds ok %d  err %d  timeout %d" % (lk.n_ok, lk.n_err, lk.n_timeout))
        elif d["data_fresh"]:
            L["link"].setText("DATA only - no FC lines (old firmware?)  |  cmds ok %d  timeout %d" % (lk.n_ok, lk.n_timeout))
        else:
            L["link"].setText("no telemetry  |  cmds ok %d  err %d  timeout %d" % (lk.n_ok, lk.n_err, lk.n_timeout))

        st = lk.status
        if st:
            L["store"].setText("%s | cal=%s mekf=%s | seq %s slots %s%s" % (
                st.get("source", "?"), st.get("imu_cal", "?"), st.get("mekf", "?"),
                st.get("seq", "?"), st.get("slots", "?"),
                "  | UNSAVED CHANGES" if lk.fw_dirty else ""))
        else:
            L["store"].setText("(no STATUS yet)")
        L["prompt"].setText(lk.cal_prompt or "--")

        if s is not None and d["data_fresh"]:
            a = np.array((s.ax, s.ay, s.az)) * d["scale_a"]
            g = np.array((s.gx, s.gy, s.gz)) * d["scale_g_dps"]
            L["acc"].setText("%+8.4f %+8.4f %+8.4f  |a|=%.4f" % (a[0], a[1], a[2], float(np.linalg.norm(a))))
            L["gyr"].setText("%+8.3f %+8.3f %+8.3f" % tuple(g))
        else:
            L["acc"].setText("--")
            L["gyr"].setText("--")

        dq = d["dash_q"]
        L["dq"].setText("%+.4f %+.4f %+.4f %+.4f" % tuple(dq))
        db = d["dash_bias_dps"]
        L["dbias"].setText("%+8.4f %+8.4f %+8.4f dps" % tuple(db))

        if fc is not None and d["fc_fresh"]:
            L["fstate"].setText("%s / %s" % (_name(MEKF_STATE_NAMES, fc.state), _name(MEKF_EVENT_NAMES, fc.event)))
            L["fq"].setText("%+.4f %+.4f %+.4f %+.4f" % tuple(fc.q))
            L["fe"].setText("%+7.2f %+7.2f %+7.2f deg" % tuple(fc.euler_deg))
            L["fbias"].setText("%+8.4f %+8.4f %+8.4f dps" % tuple(fc.bias_dps))
            if fc.state == 1 and d["dash_running"]:
                tilt, yaw = attitude_difference(fc.q, dq)
                L["delta"].setText("tilt %.2f deg   yaw %+.2f deg" % (tilt, yaw))
                L["delta"].setStyleSheet("color:%s;" % ("#4cd964" if tilt < 2.0 else "#ff9f40" if tilt < 5.0 else "#ff4d4d"))
            else:
                L["delta"].setText("(a filter is still calibrating)")
                L["delta"].setStyleSheet("")
            sat = fc.saturated
            L["mix"].setText("%s%s  coll %.3f  cmd R%+.3f P%+.3f Y%+.3f" % (
                _name(MIXER_STATUS_NAMES, fc.mixer_status), " (SAT)" if sat else "", fc.collective, *fc.torque))
            col = "#ff9f40" if sat else "#4cd964"
            for i in range(4):
                self.bars[i].setValue(int(round(math.sqrt(max(fc.thrust[i], 0.0)) * 1000)))
                self.bars[i].setStyleSheet("QProgressBar{border:1px solid #555;background:#16181c;}"
                                           "QProgressBar::chunk{background:%s;}" % col)
                self.mtxt[i].setText("T%.3f %4.0fus" % (fc.thrust[i], fc.pwm[i]))
        else:
            for k in ("fstate", "fq", "fe", "fbias", "delta", "mix"):
                L[k].setText("--")
            for i in range(4):
                self.bars[i].setValue(0)
                self.mtxt[i].setText("--")

        for r in self.rows:
            r.refresh()
        if lk.params_complete:
            self._rebuild_all()
        for r in self.all_rows:
            r.refresh()
        entries, self._log_pos = lk.new_log(self._log_pos)
        for ts, text in entries:
            self.log_view.appendPlainText("%s  %s" % (time.strftime("%H:%M:%S", time.localtime(ts)), text))


def attitude_difference(q_a, q_b):
    """(tilt difference [deg], yaw difference [deg]) between two body->world quaternions.
    Tilt = angle between the two gravity directions seen from the body (independent of yaw, which
    drifts separately in each filter); yaw = difference of the Euler yaw angles, wrapped."""
    ga = quat_to_rot(q_a)[2, :]
    gb = quat_to_rot(q_b)[2, :]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(ga, gb))))))
    dy = math.degrees(quat_to_euler(q_a)[2] - quat_to_euler(q_b)[2])
    dy = (dy + 180.0) % 360.0 - 180.0
    return tilt, dy


# ----------------------------------------------------------------------------------------------
# Dashboard
# ----------------------------------------------------------------------------------------------
_BaseWindow = QtWidgets.QMainWindow if HAVE_GUI else object


class Dashboard(_BaseWindow):
    SOURCE_CHOICES = ("Auto (firmware if live)", "Firmware (FC)", "Dashboard MEKF")
    INGEST_MS = 5
    VIZ_MS = 33            # ~30 FPS 3D
    PLOT_MS = 40           # ~25 FPS plot
    LABEL_MS = 100
    LINK_MS = 20           # command link pacing
    FC_FRESH_S = 0.5       # FC record younger than this counts as live
    BUDGET_S = 0.008       # max filter time per ingest tick, rest waits for the next tick

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, autoconnect=True):
        super().__init__()
        self.setWindowTitle("SORA-FC Attitude Monitoring Dashboard")
        self.resize(1280, 820)
        self.rx_queue = queue.Queue(maxsize=8192)
        self.reader = None
        self.mekf = MEKF()
        self.timebase = TimeBase()
        self.link = FirmwareLink(self._send_bytes)
        self.fc = None                   # latest FcRecord from the firmware
        self.fc_rx = 0.0                 # monotonic time it arrived
        self.data_rx = 0.0               # ... latest DATA sample
        self._synced = False             # STATUS/LIST requested since the last (re)connect
        self._link_err_seen = ""
        self._connect_t = 0.0
        self.fw_panel = None
        self.M = SENSOR_TO_BODY_PRESETS[DEFAULT_MAPPING].copy()
        self.paused = False
        self.sim_t = 0.0
        self.last = None                 # latest raw Sample
        self.n_processed = 0
        self._rate_t, self._rate_n, self.rate_hz = time.monotonic(), 0, 0.0
        self.last_error = ""
        self.flip = np.ones(3)                      # per-axis gyro sign fix (sensor frame)
        self._dbuf = collections.deque(maxlen=6)
        self._AtA = np.zeros((3, 3))
        self._Atb = np.zeros(3)
        self.drone = None
        self.plot = TelemetryPlot()
        self.mixer = MotorMixer()
        self.mixer_panel = None
        self._build_ui(port, baud)
        for ms, slot in ((self.INGEST_MS, self._ingest), (self.VIZ_MS, self._update_view),
                         (self.PLOT_MS, self._update_plot), (self.LABEL_MS, self._update_labels),
                         (self.LINK_MS, self._pump_link)):
            t = QtCore.QTimer(self)
            t.timeout.connect(slot)
            t.start(ms)
        if autoconnect:
            QtCore.QTimer.singleShot(300, self.connect_port)

    # -- UI ----------------------------------------------------------------------------------
    def _build_ui(self, port, baud):
        mono = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)
        mono.setPointSize(9)
        try:
            self.drone = Drone3D()
            left = self.drone.widget
        except Exception as exc:             # OpenGL init failure must not kill the dashboard
            self.last_error = "3D view unavailable: %s" % exc
            left = QtWidgets.QLabel("3D view unavailable (OpenGL failure)\n%s" % exc)
            left.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            left.setWordWrap(True)

        panel = QtWidgets.QWidget()
        panel.setFixedWidth(360)
        v = QtWidgets.QVBoxLayout(panel)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(4)

        # controls
        g = QtWidgets.QGridLayout()
        g.setSpacing(4)
        self.cb_port = QtWidgets.QComboBox()
        self.cb_port.setEditable(True)
        self._refresh_ports(port)
        self.cb_baud = QtWidgets.QComboBox()
        self.cb_baud.setEditable(True)
        for b in BAUD_CHOICES:
            self.cb_baud.addItem(str(b))
        self.cb_baud.setCurrentText(str(baud))
        btn = lambda text, slot: self._button(text, slot)
        self.btn_connect = btn("Connect", self.on_connect_clicked)
        self.btn_pause = btn("Pause view", self.on_pause_clicked)
        self.btn_reset = btn("Reset attitude", lambda: self.mekf.reset_attitude())
        self.btn_reinit = btn("Reinitialize MEKF", self.on_reinit)
        self.btn_clear = btn("Clear history", lambda: self.plot.clear())
        self.btn_bias = btn("Recalibrate bias", lambda: self.mekf.recalibrate_bias())
        self.sp_gyro = QtWidgets.QDoubleSpinBox()
        self.sp_gyro.setDecimals(5)
        self.sp_gyro.setRange(1e-5, 1000.0)
        self.sp_gyro.setValue(GYRO_DPS_PER_LSB)
        self.sp_acc = QtWidgets.QDoubleSpinBox()
        self.sp_acc.setDecimals(6)
        self.sp_acc.setRange(1e-6, 100.0)
        self.sp_acc.setValue(ACCEL_G_PER_LSB)
        self.cb_map = QtWidgets.QComboBox()
        self.cb_map.addItems(list(SENSOR_TO_BODY_PRESETS.keys()))
        self.cb_map.currentTextChanged.connect(self.on_mapping_changed)
        g.addWidget(QtWidgets.QLabel("Port"), 0, 0)
        g.addWidget(self.cb_port, 0, 1)
        g.addWidget(QtWidgets.QLabel("Baud"), 1, 0)
        g.addWidget(self.cb_baud, 1, 1)
        g.addWidget(self.btn_connect, 2, 0)
        g.addWidget(self.btn_pause, 2, 1)
        g.addWidget(self.btn_reset, 3, 0)
        g.addWidget(self.btn_reinit, 3, 1)
        g.addWidget(self.btn_bias, 4, 0)
        g.addWidget(self.btn_clear, 4, 1)
        g.addWidget(QtWidgets.QLabel("Gyro deg/s per LSB"), 5, 0)
        g.addWidget(self.sp_gyro, 5, 1)
        g.addWidget(QtWidgets.QLabel("Accel g per LSB"), 6, 0)
        g.addWidget(self.sp_acc, 6, 1)
        g.addWidget(QtWidgets.QLabel("Sensor axes"), 7, 0)
        g.addWidget(self.cb_map, 7, 1)
        fl = QtWidgets.QHBoxLayout()
        self.cb_flip = []
        for nm in "XYZ":
            c = QtWidgets.QCheckBox("-" + nm)
            c.stateChanged.connect(self._on_flip)
            fl.addWidget(c)
            self.cb_flip.append(c)
        g.addWidget(QtWidgets.QLabel("Flip gyro axis"), 8, 0)
        g.addLayout(fl, 8, 1)
        self.btn_autofix = self._button("Auto-fix gyro scale/signs", self.on_autofix)
        g.addWidget(self.btn_autofix, 9, 0, 1, 2)
        self.cb_src = QtWidgets.QComboBox()
        self.cb_src.addItems(list(self.SOURCE_CHOICES))
        g.addWidget(QtWidgets.QLabel("Attitude source"), 10, 0)
        g.addWidget(self.cb_src, 10, 1)
        v.addLayout(g)

        # telemetry
        tg = QtWidgets.QGridLayout()
        tg.setSpacing(2)
        self.lbl = {}
        rows = [("conn", "USB CDC"), ("port", "COM port"), ("rate", "Packet rate"), ("ts", "Timestamp"),
                ("acc", "Accel g (calibr.)"), ("gyr", "Gyro dps (calibr.)"), ("bias", "Dash MEKF bias"),
                ("roll", "Roll (shown)"), ("pitch", "Pitch (shown)"), ("yaw", "Yaw (shown)"),
                ("quat", "Quat WXYZ (shown)"), ("src", "Shown source"),
                ("gchk", "Gyro/accel gain"), ("filt", "Filter"), ("cnt", "Counters")]
        for r, (k, name) in enumerate(rows):
            tg.addWidget(QtWidgets.QLabel(name), r, 0)
            lab = QtWidgets.QLabel("--")
            lab.setFont(mono)
            lab.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
            tg.addWidget(lab, r, 1)
            self.lbl[k] = lab
        tg.setColumnStretch(1, 1)
        v.addLayout(tg)
        self.lbl_err = QtWidgets.QLabel("")
        self.lbl_err.setWordWrap(True)
        self.lbl_err.setStyleSheet("color:#ff6b6b;")
        v.addWidget(self.lbl_err)
        v.addStretch(1)

        top = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(top)
        h.setContentsMargins(0, 0, 0, 0)
        h.addWidget(left, 1)
        h.addWidget(panel, 0)
        split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        split.addWidget(top)
        split.addWidget(self.plot.widget)
        split.setStretchFactor(0, 4)
        split.setStretchFactor(1, 1)
        split.setSizes([560, 220])
        self.setCentralWidget(split)
        self.mixer_panel = MixerPanel(self.mixer, lambda: self.mekf.euler_deg[2] * math.pi / 180.0,
                                      self.mixer.reset)
        self.addDockWidget(QtCore.Qt.DockWidgetArea.RightDockWidgetArea, self.mixer_panel.dock)
        self.fw_panel = FirmwarePanel(self.link, lambda: np.degrees(self.mekf.bg))
        self.addDockWidget(QtCore.Qt.DockWidgetArea.RightDockWidgetArea, self.fw_panel.dock)
        self.tabifyDockWidget(self.mixer_panel.dock, self.fw_panel.dock)
        self.fw_panel.dock.raise_()
        self.statusBar().showMessage("Ready")

    def _button(self, text, slot):
        b = QtWidgets.QPushButton(text)
        b.clicked.connect(lambda *_: slot())
        return b

    def _refresh_ports(self, default):
        names = []
        try:
            names = [p.device for p in serial.tools.list_ports.comports()]
        except Exception:
            pass
        self.cb_port.clear()
        items = [default] + [n for n in names if n != default]
        self.cb_port.addItems(items)
        self.cb_port.setCurrentText(default)

    # -- control slots -----------------------------------------------------------------------
    def on_connect_clicked(self):
        if self.reader is not None:
            self.disconnect_port()
        else:
            self.connect_port()

    def connect_port(self):
        if self.reader is not None:
            return
        if not HAVE_SERIAL:
            self._set_error("pyserial is not installed (pip install pyserial)")
            return
        port = self.cb_port.currentText().strip()
        try:
            baud = int(self.cb_baud.currentText())
        except ValueError:
            self._set_error("Invalid baud rate")
            return
        self._drain_queue()
        self.timebase.reset()
        self.mekf.reinitialize()
        self.plot.clear()
        self.sim_t = 0.0
        self.last_error = ""
        self.link.reset()
        self.fc = None
        self._synced = False
        self._connect_t = time.monotonic()
        self.reader = SerialReader(port, baud, self.rx_queue)
        self.reader.start()
        self.btn_connect.setText("Disconnect")

    def disconnect_port(self):
        r, self.reader = self.reader, None
        if r is not None:
            r.stop()
            r.join(1.0)
        self._drain_queue()
        self.link.reset()
        self._synced = False
        self.btn_connect.setText("Connect")

    def on_pause_clicked(self):
        self.paused = not self.paused
        self.btn_pause.setText("Start view" if self.paused else "Pause view")

    def on_reinit(self):
        self.mixer.reset()
        self.mixer.set_yaw_hold(0.0)
        self.mekf.reinitialize()
        self.timebase.reset()
        self.plot.clear()
        self.sim_t = 0.0

    def on_mapping_changed(self, name):
        self.M = SENSOR_TO_BODY_PRESETS[name].copy()
        self.on_reinit()

    def _on_flip(self, *_):
        self.flip = np.array([-1.0 if c.isChecked() else 1.0 for c in self.cb_flip])
        self._diag_reset()
        self.on_reinit()

    def _diag_reset(self):
        self._dbuf.clear()
        self._AtA[:] = 0.0
        self._Atb[:] = 0.0

    def _diag_gains(self, thr=0.02):
        """Per-gyro-axis gain k_j so that d(accel_unit)/dt = -sum_j k_j (e_j w_j) x accel_unit.
        k=+1: scale and sign right. k<0: sign flipped. |k|!=1: scale error. None: not excited.
        (Rotating about the axis closest to the gravity direction, i.e. yaw when level, is invisible.)"""
        obs = [j for j in range(3) if self._AtA[j, j] > thr]
        gains = [None, None, None]
        if obs:
            A = self._AtA[np.ix_(obs, obs)] + 1e-9 * np.eye(len(obs))
            k = np.linalg.solve(A, self._Atb[obs])
            for i, j in enumerate(obs):
                gains[j] = float(k[i])
        return gains

    def on_autofix(self):
        gains = self._diag_gains()
        seen = [g for g in gains if g is not None]
        if not seen:
            self._set_error("Auto-fix needs data: tilt the board by hand (roll and pitch, ~20-60 deg/s) first")
            return
        for j, gk in enumerate(gains):
            if gk is not None:
                self.cb_flip[j].blockSignals(True)
                self.cb_flip[j].setChecked(gk < 0)
                self.cb_flip[j].blockSignals(False)
        mag = float(np.mean([abs(g) for g in seen]))
        if mag > 1e-3:
            self.sp_gyro.setValue(self.sp_gyro.value() * mag)
        self._on_flip()
        self._set_error("Applied: gyro scale x%.3f, flips %s (re-run to confirm gains ~ +1)" % (
            mag, "".join(n for n, f in zip("XYZ", self.flip) if f < 0) or "none"))

    def _diag_update(self, gyro_s, acc_s, dt):
        n = float(np.linalg.norm(acc_s))
        if n < 1e-6:
            return
        au = acc_s / n
        ax, ay, az = au
        C = np.array([[0.0, -az, ay], [az, 0.0, -ax], [-ay, ax, 0.0]])     # rows e_j x a
        buf = self._dbuf
        buf.append((au, -dt * gyro_s[:, None] * C))
        if len(buf) < buf.maxlen:
            return
        da = buf[-1][0] - buf[0][0]
        A = sum(b[1] for b in list(buf)[1:])
        self._AtA = 0.998 * self._AtA + A @ A.T
        self._Atb = 0.998 * self._Atb + A @ da

    def _set_error(self, msg):
        self.last_error = msg
        self.statusBar().showMessage(msg)

    def _drain_queue(self):
        try:
            while True:
                self.rx_queue.get_nowait()
        except queue.Empty:
            pass

    # -- telemetry processing (GUI thread) -----------------------------------------------------
    def _ingest(self):
        t_end = time.perf_counter() + self.BUDGET_S
        q = self.rx_queue
        if q.qsize() > 4000:                   # hopelessly behind: drop backlog, gap logic handles it
            self._drain_queue()
        while time.perf_counter() < t_end:
            try:
                s = q.get_nowait()
            except queue.Empty:
                break
            if isinstance(s, Sample):
                self._process_sample(s)
            elif isinstance(s, FcRecord):
                self.fc = s
                self.fc_rx = time.monotonic()
            elif isinstance(s, FwLine):
                self.link.on_line(s)

    def process_sample(self, s):
        self._process_sample(s)

    # -- firmware command link ---------------------------------------------------------------
    def _send_bytes(self, data):
        r = self.reader
        return r.send(data) if r is not None else False

    def _pump_link(self):
        r = self.reader
        now = time.monotonic()
        if r is not None and r.state == SerialReader.CONNECTED and not self._synced \
                and now - self._connect_t > 0.4:
            # Boot-time notices (CFG,LOADED ...) may have been sent before the port was opened, so ask.
            self._synced = True
            self.link.request_sync()
        self.link.pump(now)

    def _fc_fresh(self):
        return self.fc is not None and time.monotonic() - self.fc_rx < self.FC_FRESH_S

    def _att_q(self):
        """Quaternion shown in the 3D view / plot, and where it comes from."""
        mode = self.cb_src.currentText() if getattr(self, "cb_src", None) is not None else self.SOURCE_CHOICES[0]
        fc_ok = self._fc_fresh()
        if mode.startswith("Firmware") and self.fc is not None:
            return self.fc.q, "firmware (FC)"
        if mode.startswith("Auto") and fc_ok and self.fc.state == 1:
            return self.fc.q, "firmware (FC)"
        return self.mekf.q, "dashboard MEKF"

    def _process_sample(self, s):
        self.last = s
        self.data_rx = time.monotonic()
        dt, kind, gap = self.timebase.update(s.t)
        if dt is None:
            if kind == "gap" and self.mekf.state == MEKF.RUN:
                self.mekf.inflate(gap)
            if kind == "reset":              # timestamps went backwards: the FC rebooted
                self.fc = None
                self.link.request_sync()
            return
        scale_g = self.sp_gyro.value() * math.pi / 180.0
        scale_a = self.sp_acc.value()
        gyro_raw = np.array((s.gx, s.gy, s.gz)) * scale_g          # sensor frame, rad/s, no flips
        acc_s = np.array((s.ax, s.ay, s.az)) * scale_a
        self._diag_update(gyro_raw, acc_s, dt)
        gyro = self.M @ (gyro_raw * self.flip)
        acc = self.M @ acc_s
        self.mekf.process(gyro, acc, dt)
        if self.mixer_panel is not None:
            self.mixer.update(self.mekf.q, gyro - self.mekf.bg, dt, self.mixer_panel.enabled,
                              self.mekf.state == MEKF.RUN)
        self.n_processed += 1
        self.sim_t += dt
        if self.mekf.state == MEKF.RUN or self._fc_fresh():
            self.plot.push(self.sim_t, self._att_q()[0])

    # -- periodic GUI updates ------------------------------------------------------------------
    def _update_view(self):
        if self.paused or self.drone is None:
            return
        self.drone.set_orientation(self._att_q()[0])

    def _update_plot(self):
        if not self.paused:
            self.plot.refresh()

    def _conn_status(self):
        r = self.reader
        if r is None:
            return "DISCONNECTED", "#9aa0a6"
        st = r.state
        if st == SerialReader.CONNECTING:
            return "CONNECTING...", "#f0c040"
        if st == SerialReader.CONNECTED:
            now = time.monotonic()
            if r.last_rx == 0.0 or now - r.last_rx > STALE_TIMEOUT_S:
                if r.last_any_rx and now - r.last_any_rx < STALE_TIMEOUT_S:
                    return "CONNECTED - FC alive, no IMU data (calibrating?)", "#f0c040"
                return "CONNECTED - NO DATA (stale)", "#ff9f40"
            return "CONNECTED - receiving", "#4cd964"
        if st == SerialReader.LOST:
            return "CONNECTION LOST", "#ff4d4d"
        if st == SerialReader.ERROR:
            return "ERROR", "#ff4d4d"
        return st, "#9aa0a6"

    def _update_labels(self):
        now = time.monotonic()
        r = self.reader
        if r is not None:
            if now - self._rate_t >= 0.5:
                self.rate_hz = (r.valid_count - self._rate_n) / (now - self._rate_t)
                self._rate_t, self._rate_n = now, r.valid_count
            if r.state in (SerialReader.ERROR, SerialReader.LOST):
                self.last_error = r.error
                if not r.is_alive():
                    self.btn_connect.setText("Connect")
                    self.reader = None
        else:
            self.rate_hz = 0.0
            self._rate_n = 0
        text, color = self._conn_status()
        L = self.lbl
        L["conn"].setText(text)
        L["conn"].setStyleSheet("color:%s;font-weight:bold;" % color)
        L["port"].setText("%s @ %s" % (self.cb_port.currentText(), self.cb_baud.currentText()))
        L["rate"].setText("%.1f Hz" % self.rate_hz)
        s = self.last
        if s is not None:
            L["ts"].setText("%d ms" % s.t)
            a = np.array((s.ax, s.ay, s.az)) * self.sp_acc.value()
            g_dps = np.array((s.gx, s.gy, s.gz)) * self.sp_gyro.value()
            L["acc"].setText("%+7.4f %+7.4f %+7.4f" % tuple(a))
            L["gyr"].setText("%+7.3f %+7.3f %+7.3f" % tuple(g_dps))
        m = self.mekf
        bias_dps = np.degrees(m.bg)
        L["bias"].setText("%6.3f %6.3f %6.3f deg/s" % tuple(bias_dps))
        q_show, src = self._att_q()
        roll, pitch, yaw = (math.degrees(a_) for a_ in quat_to_euler(q_show))
        L["roll"].setText("%+7.2f deg" % roll)
        L["pitch"].setText("%+7.2f deg" % pitch)
        L["yaw"].setText("%+7.2f deg (drifts, no mag)" % yaw)
        L["quat"].setText("%+.4f %+.4f %+.4f %+.4f" % tuple(q_show))
        L["src"].setText(src)
        gk = self._diag_gains()
        L["gchk"].setText(" ".join("%s%s" % (n, ("%+.2f" % g) if g is not None else " n/a")
                                   for n, g in zip("XYZ", gk)))
        L["filt"].setText("%s: %s" % (m.state, m.msg))
        tb = self.timebase
        pr = self.reader.parser if self.reader else None
        L["cnt"].setText("ok %s bad %s gaps %d rej %d drop %s" % (
            pr.n_ok if pr else 0, pr.n_bad if pr else 0, tb.gaps, m.rejected,
            self.reader.dropped if self.reader else 0))
        self.lbl_err.setText(self.last_error)
        if self.mixer_panel is not None:
            self.mixer_panel.update(self.mixer.out)
        if self.fw_panel is not None:
            now_m = time.monotonic()
            self.fw_panel.update({
                "sample": s, "fc": self.fc,
                "fc_fresh": self._fc_fresh(),
                "data_fresh": s is not None and now_m - self.data_rx < STALE_TIMEOUT_S,
                "scale_a": self.sp_acc.value(), "scale_g_dps": self.sp_gyro.value(),
                "dash_q": self.mekf.q, "dash_bias_dps": np.degrees(self.mekf.bg),
                "dash_running": self.mekf.state == MEKF.RUN})
        if self.link.last_error != self._link_err_seen:      # show each command failure once
            self._link_err_seen = self.link.last_error
            if self._link_err_seen:
                self.last_error = "FC: " + self._link_err_seen
        if self.last_error:
            self.statusBar().showMessage(self.last_error)

    def closeEvent(self, ev):
        self.disconnect_port()
        super().closeEvent(ev)


# ----------------------------------------------------------------------------------------------
# Headless self-test
# ----------------------------------------------------------------------------------------------
def _simulate(omega_fn, T, hz=100, bias=(0, 0, 0), gyro_noise=0.0, acc_noise=0.0, seed=1, q0=None):
    rng = np.random.default_rng(seed)
    dt = 1.0 / hz
    qt = _IDENT_Q.copy() if q0 is None else q0.copy()
    out = []
    for k in range(int(T * hz)):
        w = np.asarray(omega_fn(k * dt), float)
        qt = quat_normalize(quat_mul(qt, quat_from_rotvec(w * dt)))
        a = -quat_to_rot(qt)[2, :] + rng.normal(0, acc_noise, 3)
        g = w + np.asarray(bias) + rng.normal(0, gyro_noise, 3)
        out.append((g, a, dt, qt.copy()))
    return out


def run_self_test():
    ok = True

    def check(name, cond, info=""):
        nonlocal ok
        ok &= bool(cond)
        print("[%s] %s %s" % ("PASS" if cond else "FAIL", name, info))

    # 1. parser on the provided samples, with awkward chunking
    raw = (b"DATA,85744,56,16,1002,-39,-36,-40,27\r\nDATA,85754,57,15,1001,-39,-36,-40,27\n"
           b"DATA,85764,54,14,1001,-39,-36,-40,27\nDATA,85774,57,18,996,-39,146,-40,27\n"
           b"DATA,85784,60,12,999,-39,-36,-40,27\nDATA,85794,59,13,996,21,-280,-40,27\n"
           b"DATA,85804,57,12,999,-100,24,-40,27\nDATA,85814,56,14,1001,-39,-97,20,27\n")
    p = TelemetryParser()
    got = []
    for i in range(0, len(raw), 7):
        got += p.feed(raw[i:i + 7])
    check("parser: 8 sample packets, chunked 7 bytes at a time", len(got) == 8 and p.n_bad == 0)
    check("parser: field values", got[0] == Sample(85744, 56, 16, 1002, -39, -36, -40, [27.0]) and got[5].gx == 21
          and got[7].gz == 20, str(got[0]))
    pp = TelemetryParser()
    got2 = pp.feed(b"DATA,1,2\nDATA,1,2,x,4,5,6,7\nDATA,1,nan,2,3,4,5,6\nhello\nDATA,5,1,2,3,4,5,6\n")
    check("parser: malformed rejected, valid kept", len(got2) == 1 and pp.n_bad == 3 and pp.n_ignored == 1)

    # 2. dt, rollover, gap
    tb = TimeBase()
    dts = [tb.update(g.t)[0] for g in got]
    check("dt = 10 ms from timestamps", all(abs(d - 0.01) < 1e-12 for d in dts[1:]) and dts[0] is None)
    tb = TimeBase()
    tb.update(2 ** 32 - 5)
    d, k, _ = tb.update(5)
    check("timestamp rollover (uint32)", k == "rollover" and abs(d - 0.01) < 1e-12)
    tb.update(100)
    d, k, g = tb.update(900)
    check("timestamp gap detected", d is None and k == "gap" and abs(g - 0.8) < 1e-9)
    d, k, _ = tb.update(50)
    check("timestamp going backwards = reset", d is None and k == "reset")

    # 3. quaternion basics
    rng = np.random.default_rng(0)
    q = _IDENT_Q.copy()
    for _ in range(10000):
        q = quat_mul(q, quat_from_rotvec(rng.normal(0, 0.1, 3)))
    q = quat_normalize(q)
    R = quat_to_rot(q)
    check("quaternion normalization", abs(np.linalg.norm(q) - 1) < 1e-12)
    check("R(q) orthonormal, det=+1", np.allclose(R @ R.T, np.eye(3), atol=1e-12) and abs(np.linalg.det(R) - 1) < 1e-12)
    Rz = quat_to_rot(quat_from_rotvec(np.array([0, 0, math.pi / 2])))
    check("90 deg about z maps x->y", np.allclose(Rz @ [1, 0, 0], [0, 1, 0], atol=1e-12))
    check("rotvec_to_rot == quat_to_rot(exp)", np.allclose(rotvec_to_rot(np.array([.3, -.2, .5])),
          quat_to_rot(quat_from_rotvec(np.array([.3, -.2, .5]))), atol=1e-12))
    e = quat_to_euler(euler_to_quat(0.3, -0.4, 2.5))
    check("euler <-> quaternion roundtrip", np.allclose(e, (0.3, -0.4, 2.5), atol=1e-12))

    # 4. stationary init from the real sample data (sensor x fwd,y left,z up -> FRD)
    M = SENSOR_TO_BODY_PRESETS[DEFAULT_MAPPING]
    f = MEKF()
    g_raw = np.array([-39.0, -36.0, -40.0]) * GYRO_DPS_PER_LSB * math.pi / 180
    a_raw = np.array([56.0, 16.0, 1002.0]) * ACCEL_G_PER_LSB
    for _ in range(300):
        f.process(M @ g_raw, M @ a_raw, 0.01)
    r, pi_, y = f.euler_deg
    check("stationary init: state RUNNING, bias = mean gyro", f.state == MEKF.RUN and np.allclose(f.bg, M @ g_raw, atol=1e-9))
    check("stationary init: level attitude from gravity", abs(r - 0.9) < 0.3 and abs(pi_ - 3.2) < 0.3 and abs(y) < 1e-6,
          "roll %.2f pitch %.2f yaw %.2f" % (r, pi_, y))

    # 5. signs of roll / pitch / yaw rotations, after a 2.5 s stationary calibration
    def run_rot(axis, rate_dps, T=2.0):
        w = np.zeros(3)
        w[axis] = math.radians(rate_dps)
        sim = _simulate(lambda t: np.zeros(3) if t < 2.5 else w, 2.5 + T, gyro_noise=0.003, acc_noise=0.003)
        f = MEKF()
        for g, a, dt, qt in sim:
            f.process(g, a, dt)
        return f.euler_deg, np.degrees(quat_to_euler(sim[-1][3]))
    for axis, nm in enumerate(("roll", "pitch", "yaw")):
        for sgn in (+1, -1):
            est, tru = run_rot(axis, sgn * 25.0)
            check("%s %+d: estimate follows truth" % (nm, sgn), np.allclose(est, tru, atol=2.5) and est[axis] * sgn > 20,
                  "est %s truth %s" % (np.round(est, 1), np.round(tru, 1)))

    # 6. gyro bias estimation: calibrate, then bias steps by 0.02 rad/s on x, y
    sim = _simulate(lambda t: np.zeros(3), 2.5, bias=(0.01, -0.02, 0.005), gyro_noise=0.003, acc_noise=0.003)
    sim2 = _simulate(lambda t: np.zeros(3), 120, bias=(0.03, 0.0, 0.005), gyro_noise=0.003, acc_noise=0.003, seed=2)
    f = MEKF()
    for g, a, dt, qt in sim + sim2:
        f.process(g, a, dt)
    err = abs(f.bg[0] - 0.03)
    check("gyro bias: calibration captured, step change tracked", err < 0.005 and abs(f.bg[1]) < 0.005,
          "bg=%s (true 0.03, 0, 0.005)" % np.round(f.bg, 4))

    # 7. accelerometer gravity update: start with a 25 deg wrong attitude
    f = MEKF()
    f.state = MEKF.RUN
    f.P = np.diag([0.5 ** 2] * 3 + [0.01 ** 2] * 3)
    a_true = -quat_to_rot(euler_to_quat(math.radians(25), math.radians(-15), 0))[2, :]
    for _ in range(300):
        f.process(np.zeros(3), a_true, 0.01)
    r, pi_, _y = f.euler_deg
    check("accel update corrects roll/pitch", abs(r - 25) < 0.5 and abs(pi_ + 15) < 0.5, "roll %.2f pitch %.2f" % (r, pi_))
    rej0 = f.rejected
    f.process(np.zeros(3), a_true * 2.0, 0.01)
    check("accel with |a|=2g rejected", f.rejected == rej0 + 1)
    q_before = f.q.copy()
    f.process(np.zeros(3), np.zeros(3), 0.01)
    check("zero accel does not corrupt the filter", np.all(np.isfinite(f.q)) and np.allclose(f.q, q_before, atol=1e-6))

    # 8. covariance symmetric and PSD after a long mixed run
    sim = _simulate(lambda t: np.zeros(3) if t < 2.5 else
                    0.5 * np.array([math.sin(t), math.cos(1.3 * t), math.sin(0.7 * t)]), 32.5,
                    bias=(0.01, 0.02, -0.01), gyro_noise=0.005, acc_noise=0.01)
    f = MEKF()
    worst = 0.0
    for i, (g, a, dt, qt) in enumerate(sim):
        f.process(g, a, dt)
        if i > 400:
            d = np.degrees(quat_to_euler(f.q)) - np.degrees(quat_to_euler(qt))
            worst = max(worst, abs(d[0]), abs(d[1]))
    ev = np.linalg.eigvalsh(f.P)
    check("covariance symmetric", np.allclose(f.P, f.P.T, atol=1e-15))
    check("covariance PSD", ev.min() > -1e-12, "min eig %.2e" % ev.min())
    check("quaternion unit norm after run", abs(np.linalg.norm(f.q) - 1) < 1e-12 and f.recoveries == 0)
    check("dynamic tracking: worst roll/pitch error < 5 deg", worst < 5, "%.2f deg" % worst)

    # 10. model / display conventions: +pitch = nose up, +roll = right wing down, +yaw = nose east
    Rp = quat_to_rot(euler_to_quat(0, math.radians(30), 0))
    Rr = quat_to_rot(euler_to_quat(math.radians(30), 0, 0))
    Ry = quat_to_rot(euler_to_quat(0, 0, math.radians(30)))
    check("+pitch: nose goes UP in view", (NED_TO_DISPLAY @ Rp @ [1, 0, 0])[2] > 0.4)
    check("+roll: right wing goes DOWN in view", (NED_TO_DISPLAY @ Rr @ [0, 1, 0])[2] < -0.4)
    check("+yaw: nose turns east (view y<0)", (NED_TO_DISPLAY @ Ry @ [1, 0, 0])[1] < -0.4)
    check("identity attitude: model upright (-z_body is view up)", (NED_TO_DISPLAY @ [0, 0, -1])[2] > 0.99)
    check("NED->view is a proper rotation", abs(np.linalg.det(NED_TO_DISPLAY) - 1) < 1e-12)

    # 11. serial thread: loopback, then forced disconnect
    if HAVE_SERIAL:
        qq = queue.Queue()
        rd = SerialReader("loop://", 115200, qq)
        rd.start()
        t0 = time.time()
        while rd.state == SerialReader.CONNECTING and time.time() - t0 < 2:
            time.sleep(0.01)
        rd.ser.write(b"DATA,10,1,2,3,4,5,6,7\nDATA,20,1,2,3,4,5,6,7\nDATA,2")
        time.sleep(0.3)
        rd.ser.write(b"0,garbage\nDATA,30,1,2,3,4,5,6,7\n")
        time.sleep(0.3)
        n = qq.qsize()
        check("serial thread: packets via queue, partial read handled", n == 3 and rd.parser.n_bad == 1,
              "queued %d bad %d" % (n, rd.parser.n_bad))
        rd.ser.close()                         # simulate USB unplug
        rd.join(2)
        check("serial thread: disconnect detected, thread ends", rd.state == SerialReader.LOST and not rd.is_alive(), rd.error)
        bad = SerialReader("/dev/this_port_does_not_exist", 115200, queue.Queue())
        bad.start()
        bad.join(2)
        check("serial thread: unavailable port reported", bad.state == SerialReader.ERROR, bad.error)

    # 9b. motor mixer: torque signs, hover, tilt compensation, saturation, closed loop
    FMAX, ARM, YAWC = 8.0, 0.12, 0.02

    def torques(f):
        tx = sum(-m[3] * ARM * FMAX * fi for m, fi in zip(MOTORS, f))      # tau = r x F, F up = -z
        ty = sum(+m[2] * ARM * FMAX * fi for m, fi in zip(MOTORS, f))
        tz = sum(m[4] * YAWC * FMAX * fi for m, fi in zip(MOTORS, f))
        return np.array([tx, ty, tz])
    for k, nm in enumerate(("roll", "pitch", "yaw")):
        u = np.zeros(3)
        u[k] = 0.05
        t = torques(0.5 + MIX_MATRIX @ u)
        check("mixer: +%s command gives +%s torque only" % (nm, nm),
              t[k] > 0 and np.allclose(np.delete(t, k), 0, atol=1e-12) and abs(sum(MIX_MATRIX @ u)) < 1e-12, str(np.round(t, 4)))
    mx = MotorMixer()
    o = mx.update(_IDENT_Q, np.zeros(3), 0.002)
    check("mixer: level hover = equal motors, speed = sqrt(1/TWR)",
          np.allclose(o.speed, math.sqrt(1 / 2.0)) and np.allclose(o.thrust, 0.5) and not o.saturated,
          "speed %.3f rpm %.0f pwm %.0f" % (o.speed[0], o.rpm[0], o.pwm[0]))
    check("mixer: total thrust = weight at hover", abs(o.thrust.sum() * FMAX - 4 * FMAX / 2.0) < 1e-9)
    qt = euler_to_quat(math.radians(30), 0, 0)
    mx = MotorMixer()
    mx.cfg.ki_rp = mx.cfg.kp_rp = mx.cfg.kd_rp = 0.0
    o = mx.update(qt, np.zeros(3), 0.002)
    vert = o.thrust.sum() * math.cos(math.radians(30)) / 4
    check("mixer: tilt compensation keeps vertical thrust = weight/4 per motor", abs(vert - 0.5) < 1e-9, "%.4f" % vert)
    mx = MotorMixer()
    o = mx.update(euler_to_quat(math.radians(40), 0, math.radians(120)), np.zeros(3), 0.002)
    check("mixer: saturation resolved, outputs within limits",
          o.saturated and o.thrust.min() >= mx.cfg.idle_thrust - 1e-12 and o.thrust.max() <= 1 + 1e-12)
    o = mx.update(euler_to_quat(math.radians(75), 0, 0), np.zeros(3), 0.002)
    check("mixer: tilt limit -> idle", o.status.startswith("TILT") and np.allclose(o.thrust, mx.cfg.idle_thrust))
    Inertia = np.diag([0.003, 0.003, 0.005])
    for q0, nm in ((euler_to_quat(math.radians(25), math.radians(-15), math.radians(30)), "mixed +roll -pitch +yaw"),
                   (euler_to_quat(math.radians(-25), math.radians(15), math.radians(-30)), "mixed -roll +pitch -yaw")):
        mx = MotorMixer()
        qs, w = q0.copy(), np.zeros(3)
        dt = 0.002
        worst_f = 0.0
        for _ in range(int(5.0 / dt)):
            o = mx.update(qs, w, dt)
            w = w + dt * np.linalg.solve(Inertia, torques(o.thrust) - np.cross(w, Inertia @ w))
            qs = quat_normalize(quat_mul(qs, quat_from_rotvec(w * dt)))
        r_, p_, y_ = np.degrees(quat_to_euler(qs))
        check("closed loop (%s): returns to level, yaw held" % nm, max(abs(r_), abs(p_), abs(y_)) < 0.5 and
              np.linalg.norm(w) < 0.01, "roll %.2f pitch %.2f yaw %.2f" % (r_, p_, y_))

    # 12. performance of the filter itself
    f = MEKF()
    g_, a_ = np.array([0.01, -0.02, 0.005]), np.array([0.0, 0.0, -1.0])
    for _ in range(300):
        f.process(g_, a_, 0.002)
    t0 = time.perf_counter()
    for _ in range(5000):
        f.process(g_, a_, 0.002)
    us = (time.perf_counter() - t0) / 5000 * 1e6
    check("MEKF cost per sample (500 Hz budget = 2000 us)", us < 1000, "%.0f us" % us)

    # 5. firmware protocol: FC record, text lines, command link (canned lines captured from the real firmware)
    fc_line = "FC,6000,1,3,1909,2926,367,9995,166,256,28,202,1,-54,4,1,479,521,462,542,1692,1722,1680,1736,501,-9,-30,-1"
    data_line = "DATA,6000,28,33,1000,8,-214,-39,27"
    pk = TelemetryParser()
    pkt = (data_line + "\r\n" + fc_line + "\r\n").encode()
    got5 = []
    for i in range(0, len(pkt), 5):
        got5 += pk.feed_all(pkt[i:i + 5])
    check("parser: DATA + FC in one packet, chunked 5 bytes", len(got5) == 2 and isinstance(got5[0], Sample)
          and isinstance(got5[1], FcRecord) and pk.n_bad == 0)
    fc = got5[1]
    check("FC fields decoded with correct units",
          fc.t == 6000 and fc.state == 1 and abs(fc.euler_deg[0] - 1.909) < 1e-9 and abs(fc.euler_deg[1] - 2.926) < 1e-9
          and abs(fc.q[0] - 0.9995) < 1e-3 and abs(np.linalg.norm(fc.q) - 1) < 1e-12
          and abs(fc.bias_dps[2] + 0.054) < 1e-9 and abs(fc.thrust[0] - 0.479) < 1e-9 and fc.pwm[3] == 1736
          and fc.active and not fc.saturated and abs(fc.collective - 0.501) < 1e-9 and abs(fc.torque[1] + 0.03) < 1e-9
          and MIXER_STATUS_NAMES[fc.mixer_status] == "HOVERING" and MEKF_EVENT_NAMES[fc.event] == "RUNNING", str(fc.euler_deg))
    bad = TelemetryParser()
    bad.feed_all(b"FC,1,2,3\nFC," + b"1," * 26 + b"1\nFC,1,1,3,0,0,0,5000,0,0,0,0,0,0,4,1,0,0,0,0,0,0,0,0,0,0,0,0\n"
                 b"FC,1,1,3,0,0,0,10000,0,0,0,0,0,0,4,x,0,0,0,0,0,0,0,0,0,0,0,0\n")
    check("parser: bad FC lines rejected (short, non-unit quaternion, non-integer)", bad.n_bad == 4 and bad.n_fc == 0)
    old = TelemetryParser()
    o = old.feed((data_line + "\r\n" + fc_line + "\r\nOK,SAVE\r\n").encode())
    check("legacy feed(): DATA only, other lines ignored", len(o) == 1 and old.n_ignored == 2)
    ml = TelemetryParser().feed_all(b"OK,SET,mix.kp_rp,0.42,UNSAVED\r\n[CAL:WAIT_FACE]: POS_X,Orient this face upward and press the button\r\n"
                                    b"CFG,SAVED,seq=1\r\nCAL,COMPLETE,Calibration successful\r\nrandom text\r\n")
    check("parser: firmware text lines classified",
          [m.kind for m in ml] == ["OK", "CALPROMPT", "CFG", "CAL"], str([m.kind for m in ml]))
    check("enum name tables have the firmware's sizes", len(MEKF_EVENT_NAMES) == 14 and len(MIXER_STATUS_NAMES) == 6
          and len(MEKF_STATE_NAMES) == 2)
    t_, y_ = attitude_difference(_IDENT_Q, _IDENT_Q)
    t2, y2 = attitude_difference(euler_to_quat(math.radians(10), 0, 0), _IDENT_Q)
    t3, y3 = attitude_difference(euler_to_quat(0, 0, math.radians(30)), _IDENT_Q)
    check("attitude_difference: tilt ignores yaw, yaw wraps", t_ < 1e-6 and abs(t2 - 10) < 1e-6 and t3 < 1e-6 and abs(y3 - 30) < 1e-6
          and abs(attitude_difference(euler_to_quat(0, 0, math.radians(179)), euler_to_quat(0, 0, math.radians(-179)))[1] + 2) < 1e-6)

    sent = []
    lk = FirmwareLink(lambda b: (sent.append(b.decode()) or True))
    lk.request_sync()
    lk.set_param("mix.kp_rp", 0.42)
    lk.set_param("mix.use_integral", True)
    lk.pump(0.0)
    lk.pump(0.001)
    check("link: one command in flight, the rest wait", sent == ["STATUS\n"] and len(lk.queue) == 3)
    L_ = lambda txt: lk.on_line(TelemetryParser.parse_any(txt))
    L_("ERR,CAL_BIAS,motion detected; previous bias kept")           # asynchronous notice, not a reply
    check("link: asynchronous ERR does not complete the command", lk.inflight is not None and lk.inflight.kind == "STATUS")
    lk.inflight.kind = "SET"
    L_("ERR,SAVE_FAILED,flash write failed or storage unavailable")  # auto-save notice while a SET is in flight
    check("link: an error a SET cannot return is not taken as its reply", lk.inflight is not None)
    lk.inflight.kind = "STATUS"
    L_("STATUS,imu_cal=complete,source=flash,mekf=running,dirty=0,store=ok,seq=3,slots=3/512")
    check("link: STATUS parsed, command completed", lk.inflight is None and lk.status["seq"] == "3" and not lk.fw_dirty)
    lk.pump(0.1)
    L_("PARAM,mix.kp_rp,0.3,0,10")
    L_("PARAM,mix.use_integral,0,0,1,bool")
    L_("PARAM,mekf.bias_x,0.000764186,-50,50")
    L_("OK,LIST,3")
    check("link: LIST fills the parameter table", lk.params_complete and len(lk.params) == 3 and lk.params["mix.use_integral"].is_bool
          and abs(lk.value("mekf.bias_x") - 0.000764186) < 1e-12 and lk.inflight is None)
    lk.pump(0.2)
    check("link: SET sent with full precision", sent[-1] == "SET mix.kp_rp 0.42\n")
    L_("OK,SET,mix.kp_rp,0.42,UNSAVED")
    check("link: SET reply updates value and marks unsaved", abs(lk.value("mix.kp_rp") - 0.42) < 1e-12 and lk.fw_dirty)
    lk.pump(0.3)
    check("link: bool sent as 1", sent[-1] == "SET mix.use_integral 1\n")
    L_("ERR,RANGE,mix.use_integral,0.0,1.0")
    check("link: ERR reply fails the command and records it", lk.inflight is None and "RANGE" in lk.last_error and lk.n_err == 1)
    lk.save()
    lk.pump(0.4)
    L_("CFG,SAVED,seq=4")                                           # notice precedes the OK,SAVE reply
    check("link: CFG,SAVED triggers a resync and clears dirty", not lk.fw_dirty and any(c.kind == "LIST" for c in lk.queue)
          and lk.inflight is not None and lk.inflight.kind == "SAVE")
    L_("OK,SAVE")
    check("link: SAVE completes", lk.inflight is None)
    lk.queue.clear()
    lk.get("mix.kp_rp")
    lk.pump(1.0)
    lk.pump(1.0 + FirmwareLink.DEFAULT_TIMEOUT_S + 0.1)
    check("link: timeout releases the link", lk.inflight is None and lk.n_timeout == 1)
    long_ok = lk.submit("SET", "SET " + "x" * 120)
    check("link: over-long command refused (firmware line buffer is 96 bytes)", long_ok is False)
    dead = FirmwareLink(lambda b: False)
    dead.request_sync()
    dead.pump(0.0)
    check("link: not connected -> queue dropped, nothing stuck", dead.inflight is None and not dead.queue)

    print("SELF-TEST", "PASSED" if ok else "FAILED")
    return ok


# ----------------------------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description="SORA-FC attitude monitoring dashboard")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--no-autoconnect", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args(argv)
    if args.selftest:
        return 0 if run_self_test() else 1
    if not HAVE_GUI:
        print("GUI libraries missing (%s).\nInstall: pip install numpy pyserial pyqtgraph PyQt6 PyOpenGL"
              % GUI_IMPORT_ERROR)
        return 1
    app = pg.mkQApp("SORA-FC Dashboard")
    app.setStyle("Fusion")
    pal = QtGui.QPalette()
    R = QtGui.QPalette.ColorRole
    for role, c in ((R.Window, (30, 32, 36)), (R.WindowText, (220, 222, 226)), (R.Base, (22, 24, 28)),
                    (R.AlternateBase, (36, 38, 42)), (R.Text, (220, 222, 226)), (R.Button, (48, 51, 58)),
                    (R.ButtonText, (230, 232, 236)), (R.Highlight, (60, 120, 200)),
                    (R.HighlightedText, (255, 255, 255)), (R.ToolTipBase, (40, 40, 44)),
                    (R.ToolTipText, (230, 230, 230))):
        pal.setColor(role, QtGui.QColor(*c))
    app.setPalette(pal)
    win = Dashboard(args.port, args.baud, autoconnect=not args.no_autoconnect)
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
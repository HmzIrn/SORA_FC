"""
Minimal stateful stand-in for PyQt6 / pyqtgraph / pyserial so the REAL dashboard code (widgets'
handlers, timers' slots, the firmware panel) can run headless in CI or on a machine without Qt.
It does not draw anything; it records state (text, values, enabled flags) and fires signals, which is
enough to click buttons and edit spin boxes programmatically and read back the labels.
"""
import os
import sys
import types
import struct
import subprocess
# select/fcntl/termios are POSIX-only; they are imported lazily (PipeSerial) so that the static checks
# and the module import itself also work on Windows.


class Signal:
    def __init__(self):
        self.slots = []

    def connect(self, fn):
        self.slots.append(fn)

    def emit(self, *a):
        for fn in list(self.slots):
            fn(*a)


class Any:
    """Permissive widget: unknown attributes are no-op callables, known ones keep state."""
    _SIGNALS = ("clicked", "valueChanged", "toggled", "stateChanged", "currentTextChanged", "textChanged",
                "timeout", "itemChanged", "returnPressed", "triggered", "currentIndexChanged")

    def __init__(self, *a, **k):
        self.__dict__["_st"] = {"text": a[0] if a and isinstance(a[0], str) else "", "value": 0.0, "checked": False,
                                "items": [], "cur": None, "enabled": True, "min": -1e9, "max": 1e9,
                                "block": False, "style": "", "children": [], "range": None}
        self.__dict__["_attrs"] = {}

    # --- state ---------------------------------------------------------------------------------
    def setText(self, t): self._st["text"] = str(t)
    def text(self): return self._st["text"]
    def setPlainText(self, t): self._st["text"] = str(t)
    def appendPlainText(self, t): self._st["text"] += str(t) + "\n"
    def toPlainText(self): return self._st["text"]
    def setStyleSheet(self, s): self._st["style"] = s
    def styleSheet(self): return self._st["style"]
    def setEnabled(self, b): self._st["enabled"] = bool(b)
    def isEnabled(self): return self._st["enabled"]
    def blockSignals(self, b): self._st["block"] = bool(b)
    def setDecimals(self, d): self._st["dec"] = d
    def setSingleStep(self, s): pass
    def setKeyboardTracking(self, b): pass

    def setRange(self, lo, hi):
        self._st["min"], self._st["max"] = lo, hi
        self._st["value"] = min(max(self._st["value"], lo), hi)

    def minimum(self): return self._st["min"]
    def maximum(self): return self._st["max"]

    def setValue(self, v):
        v = min(max(float(v), self._st["min"]), self._st["max"])
        changed = v != self._st["value"]
        self._st["value"] = v
        if changed and not self._st["block"]:
            self.valueChanged.emit(v)

    def value(self): return self._st["value"]

    def setChecked(self, b):
        b = bool(b)
        changed = b != self._st["checked"]
        self._st["checked"] = b
        if changed and not self._st["block"]:
            self.toggled.emit(b)
            self.stateChanged.emit(2 if b else 0)

    def isChecked(self): return self._st["checked"]

    def addItem(self, t): self._st["items"].append(str(t))
    def addItems(self, ts): self._st["items"].extend(str(t) for t in ts)
    def clear(self): self._st["items"] = []; self._st["cur"] = None
    def setCurrentText(self, t): self._st["cur"] = str(t)

    def currentText(self):
        if self._st["cur"] is not None:
            return self._st["cur"]
        return self._st["items"][0] if self._st["items"] else ""

    def setCurrentIndex(self, i):
        if 0 <= i < len(self._st["items"]):
            self._st["cur"] = self._st["items"][i]

    def addWidget(self, w, *a, **k): self._st["children"].append(w)
    def addLayout(self, w, *a, **k): self._st["children"].append(w)
    def addTab(self, w, name): self._st["children"].append(w)
    def statusBar(self): return self._attrs.setdefault("__statusbar", Any())
    def showMessage(self, *a): self._st["status"] = a[0] if a else ""

    # --- generic -------------------------------------------------------------------------------
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        a = self.__dict__["_attrs"]
        if name not in a:
            a[name] = Signal() if name in self._SIGNALS else Any()
        return a[name]

    def __call__(self, *a, **k): return Any()
    def __or__(self, o): return Any()
    def __ror__(self, o): return Any()
    def __iter__(self): return iter(())
    def __bool__(self): return True

    def __class_getitem__(cls, item): return cls


class _Meta(type):
    def __getattr__(cls, name):          # QtWidgets.QMessageBox.StandardButton.Yes etc.
        if name.startswith("__"):
            raise AttributeError(name)
        return Any()


def make_class(name):
    return _Meta(name, (Any,), {})


class _Module(types.ModuleType):
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        c = make_class(name)
        setattr(self, name, c)
        return c


class FakeFontDb:
    class SystemFont:
        FixedFont = 1

    @staticmethod
    def systemFont(*a):
        return Any()


class PipeSerial:
    """pyserial-like object over a subprocess (the fake STM32)."""

    def __init__(self, cmd, env):
        if sys.platform == "win32":
            raise RuntimeError("the dynamic scenario needs Linux/macOS/WSL (POSIX pipes)")
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env, bufsize=0)
        self.fd = self.proc.stdout.fileno()
        self.timeout = 0.05
        self.closed = False

    @property
    def in_waiting(self):
        import fcntl
        import termios
        buf = struct.pack("i", 0)
        try:
            return struct.unpack("i", fcntl.ioctl(self.fd, termios.FIONREAD, buf))[0]
        except OSError:
            return 0

    def read(self, n=1):
        import select
        r, _, _ = select.select([self.fd], [], [], self.timeout)
        if not r:
            return b""
        d = os.read(self.fd, n)
        if not d:
            raise OSError("device closed (process exited)")
        return d

    def write(self, data):
        self.proc.stdin.write(data)
        self.proc.stdin.flush()
        return len(data)

    def reset_input_buffer(self):
        pass

    def close(self):
        if not self.closed:
            self.closed = True
            try:
                self.proc.stdin.close()
            except Exception:
                pass
            try:
                self.proc.wait(timeout=2)
            except Exception:
                self.proc.kill()


def install(serial_factory):
    """Register fake pyqtgraph / PyQt / pyserial modules. serial_factory(port, **kw) -> PipeSerial."""
    qt = _Module("pyqtgraph.Qt")
    qt.QtCore = _Module("QtCore")
    qt.QtGui = _Module("QtGui")
    qt.QtGui.QFontDatabase = FakeFontDb
    qt.QtWidgets = _Module("QtWidgets")
    pg = _Module("pyqtgraph")
    pg.Qt = qt
    gl = _Module("pyqtgraph.opengl")
    pg.opengl = gl
    sys.modules.update({"pyqtgraph": pg, "pyqtgraph.Qt": qt, "pyqtgraph.opengl": gl})

    ser = types.ModuleType("serial")
    ser.serial_for_url = serial_factory
    tools = types.ModuleType("serial.tools")
    lp = types.ModuleType("serial.tools.list_ports")
    lp.comports = lambda: []
    tools.list_ports = lp
    ser.tools = tools
    sys.modules.update({"serial": ser, "serial.tools": tools, "serial.tools.list_ports": lp})

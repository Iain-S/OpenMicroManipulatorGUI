"""Mock serial objects for running without hardware."""
import re
from collections import deque


class FakeSerial:
    def __init__(self):
        self.is_open = True
        self._rx = deque()  # bytes waiting to be read
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._is_moving = False

    @property
    def in_waiting(self):
        return len(self._rx)

    def write(self, data: bytes):
        cmd = data.decode("ascii", errors="ignore").strip()
        cmd_upper = cmd.upper()

        if cmd_upper == "M58":
            self._enqueue_line("v1.0.2")
            self._enqueue_line("ok")
        elif cmd_upper == "M50":
            self._enqueue_line(f"X{self._x:.6f} Y{self._y:.6f} Z{self._z:.6f}")
            self._enqueue_line("ok")
        elif cmd_upper == "M51":
            self._enqueue_line("A0.000000 B0.000000 C0.000000")
            self._enqueue_line("ok")
        elif cmd_upper == "M53":
            # wait_for_stop expects "1" for stopped.
            self._enqueue_line("0" if self._is_moving else "1")
            self._enqueue_line("ok")
            self._is_moving = False
        elif cmd_upper.startswith("M56"):
            # calibrate_joint parses 3 comma-separated columns.
            self._enqueue_line("0.0,0.0,100")
            self._enqueue_line("45.0,43.0,200")
            self._enqueue_line("90.0,87.0,300")
            self._enqueue_line("ok")
        elif cmd_upper.startswith("M57"):
            self._enqueue_line("state:idle")
            self._enqueue_line("ok")
        elif cmd_upper.startswith("M55"):
            self._enqueue_line("ok")
        elif cmd_upper in ("M17", "M18"):
            self._enqueue_line("ok")
        elif cmd_upper.startswith("M204"):
            self._enqueue_line("ok")
        elif cmd_upper.startswith("G0"):
            x_match = re.search(r"\bX([-+]?\d*\.?\d+)", cmd_upper)
            y_match = re.search(r"\bY([-+]?\d*\.?\d+)", cmd_upper)
            z_match = re.search(r"\bZ([-+]?\d*\.?\d+)", cmd_upper)
            if x_match:
                self._x = float(x_match.group(1))
            if y_match:
                self._y = float(y_match.group(1))
            if z_match:
                self._z = float(z_match.group(1))
            self._is_moving = True
            self._enqueue_line("ok")
        elif cmd_upper.startswith("G24"):
            x_match = re.search(r"\bX([-+]?\d*\.?\d+)", cmd_upper)
            y_match = re.search(r"\bY([-+]?\d*\.?\d+)", cmd_upper)
            z_match = re.search(r"\bZ([-+]?\d*\.?\d+)", cmd_upper)
            if x_match:
                self._x = float(x_match.group(1))
            if y_match:
                self._y = float(y_match.group(1))
            if z_match:
                self._z = float(z_match.group(1))
            self._is_moving = False
            self._enqueue_line("ok")
        elif cmd_upper.startswith("G4"):
            self._enqueue_line("ok")
        elif cmd_upper.startswith("G28"):
            self._x = 0.0
            self._y = 0.0
            self._z = 0.0
            self._is_moving = False
            self._enqueue_line("ok")
        else:
            self._enqueue_line("error:unsupported")

    def _enqueue_line(self, line: str):
        for ch in (line + "\n").encode("ascii"):
            self._rx.append(bytes([ch]))

    def read(self, n=1):
        if not self._rx:
            return b""
        return self._rx.popleft()

    def flush(self):
        pass

    def close(self):
        self.is_open = False

from __future__ import annotations

# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------
import queue
import re
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable

import numpy as np
import serial
from colorama import Fore, Style
from PySide6.QtCore import QThread

from ommg.hardware.mocks import FakeSerial

# --- SerialInterface --------------------------------------------------------------------------------------------------

@dataclass
class _CommandRequest:
    cmd: str
    timeout_s: float
    done: threading.Event = field(default_factory=threading.Event)
    response_string: str = ""
    response_status: SerialInterface.ReplyStatus | None = None
    response_error_msg: str = ""


class _SerialActorThread(QThread):
    def __init__(self, interface: SerialInterface):
        super().__init__()
        self.interface: SerialInterface = interface
        self._stop_event = threading.Event()
        self._request_queue: queue.Queue[_CommandRequest] = queue.Queue()
        self._active_request: _CommandRequest | None = None
        self._active_deadline = 0.0
        self._active_lines: list[str] = []
        self._line_buffer = ""

    def stop(self):
        self._stop_event.set()
        self.wait(1000)

    def enqueue(self, request: _CommandRequest):
        self._request_queue.put(request)

    def run(self):
        while not self._stop_event.is_set():
            if not self.interface._is_serial_open():
                self.interface.connect(self.interface.reconnect_timeout)
                if not self.interface._is_serial_open():
                    time.sleep(0.2)
                    continue

            if self._active_request is None:
                self._start_next_request_if_available()

            if self._active_request is not None and time.time() >= self._active_deadline:
                print(f"{Fore.MAGENTA}[SerialInterface] Command timeout, device didn't reply in time{Style.RESET_ALL}")
                self._finish_active_request(SerialInterface.ReplyStatus.TIMEOUT, "")
                continue

            try:
                if self.interface.serial is not None and self.interface.serial.in_waiting:
                    char = self.interface.serial.read(1).decode("ascii", errors="ignore")
                    if char in ("\n", "\r"):
                        if self._line_buffer:
                            self._handle_line(self._line_buffer)
                            self._line_buffer = ""
                    else:
                        self._line_buffer += char
                else:
                    time.sleep(0.001)
            except (serial.SerialException, OSError) as e:
                self._handle_disconnect(e)

    def _start_next_request_if_available(self):
        try:
            request = self._request_queue.get_nowait()
        except queue.Empty:
            return

        serial_port = self.interface.serial
        if serial_port is None:
            request.response_status = SerialInterface.ReplyStatus.ERROR
            request.response_error_msg = "Serial port is not connected"
            request.done.set()
            return

        try:
            serial_port.write(request.cmd.encode("ascii"))
            serial_port.flush()
            self._active_request = request
            self._active_deadline = time.time() + request.timeout_s
            self._active_lines = []
        except (serial.SerialException, OSError) as e:
            request.response_status = SerialInterface.ReplyStatus.ERROR
            request.response_error_msg = str(e)
            request.done.set()
            self._handle_disconnect(e)

    def _handle_line(self, line: str):
        log_level, log_msg = self.interface._check_log_msg(line)

        if log_level is not None:
            if self.interface.log_message_callback:
                self.interface.log_message_callback(log_level, log_msg)
            return

        if self._active_request is None:
            if self.interface.unsolicited_msg_callback:
                self.interface.unsolicited_msg_callback(line)
            return

        line_lower = line.lower()
        if line_lower.startswith("ok"):
            self._finish_active_request(SerialInterface.ReplyStatus.OK, "")
        elif line_lower.startswith("busy"):
            self._finish_active_request(SerialInterface.ReplyStatus.BUSY, "")
        elif line_lower.startswith("error"):
            parts = line.split(":", 1)
            error_msg = parts[1].strip() if len(parts) > 1 else ""
            self._finish_active_request(SerialInterface.ReplyStatus.ERROR, error_msg)
        else:
            self._active_lines.append(line)

    def _finish_active_request(self, status: SerialInterface.ReplyStatus, error_msg: str):
        request = self._active_request
        if request is None:
            return

        response = ""
        if self._active_lines:
            response = "\n".join(self._active_lines) + "\n"

        request.response_status = status
        request.response_error_msg = error_msg
        request.response_string = response
        request.done.set()

        if self.interface.command_msg_callback:
            self.interface.command_msg_callback(response, status, error_msg)

        self._active_request = None
        self._active_deadline = 0.0
        self._active_lines = []

    def _handle_disconnect(self, error: Exception):
        print(f"{Fore.MAGENTA}[SerialInterface] Lost connection: {error}{Style.RESET_ALL}")

        if self._active_request is not None:
            self._finish_active_request(SerialInterface.ReplyStatus.ERROR, str(error))

        try:
            if self.interface.serial is not None and self.interface.serial.is_open:
                self.interface.serial.close()
        except Exception:
            pass

        self.interface.serial = None

class SerialInterface:

    class ReplyStatus(Enum):
        OK = 'ok'
        ERROR = 'error'
        TIMEOUT = 'timeout'
        BUSY = 'busy'

    class LogLevel(Enum):
        DEBUG = 'debug'
        INFO = 'info'
        WARNING = 'warning'
        ERROR = 'error'

    # Static mapping from prefix to LogLevel
    log_level_prefix_map = {
        "D)": LogLevel.DEBUG,
        "I)": LogLevel.INFO,
        "W)": LogLevel.WARNING,
        "E)": LogLevel.ERROR,
    }

    def __init__(self, port: str, baud_rate: int = 115200,
                 command_msg_callback: Callable[[str, ReplyStatus | None, str], None] | None = None,
                 log_msg_callback: Callable[[LogLevel, str], None] | None = None,
                 unsolicited_msg_callback: Callable[[str], None] | None = None,
                 reconnect_timeout: int = 5):
        """
        Initializes the serial connection and starts background reader.
        :param port: Serial port name (e.g., 'COM3' or '/dev/ttyUSB0').
        :param baud_rate: Serial baud rate.
        :param log_msg_callback: called when a log message is received
        :param unsolicited_msg_callback: Optional function to call with unsolicited messages.
        """
        self.port = port
        self.baud_rate = baud_rate
        self.reconnect_timeout = reconnect_timeout
        self.serial: serial.Serial | FakeSerial | None = None  # initialized on connect

        self.command_msg_callback = command_msg_callback
        self.log_message_callback = log_msg_callback
        self.unsolicited_msg_callback = unsolicited_msg_callback

        self._actor: _SerialActorThread | None = None

        self.connect(self.reconnect_timeout)

        # Start actor thread that owns command dispatch + serial reads.
        self._actor = _SerialActorThread(self)
        self._actor.start()

    def connect(self, timeout: float) -> bool:
        """
        Try to open the serial port. Retry until timeout expires.
        """
        deadline = time.time() + timeout
        print(Fore.MAGENTA, end='')
        print(f"[SerialInterface] Connecting to port '{self.port}'...", end='')
        while time.time() < deadline:
            try:
                # Special case
                if self.port == "mock":
                    self.serial = FakeSerial()
                else:
                    self.serial = serial.Serial(self.port, self.baud_rate, timeout=2)
                print(" [OK]")
                print(Style.RESET_ALL, end='')
                return True
            except (serial.SerialException, OSError):
                print('.', end='')
                time.sleep(0.2)

        print(f" [FAILED] Timeout after {timeout} seconds.")
        print("[SerialInterface] Connection is permanently closed")
        print(Style.RESET_ALL, end='')
        self.serial = None
        return False

    def _check_log_msg(self, msg: str) -> tuple[LogLevel | None, str]:
        if len(msg) < 2:
            return None, ''
        return self.log_level_prefix_map.get(msg[:2]), msg[2:]

    def _is_serial_open(self):
        return self.serial is not None and self.serial.is_open

    def send_command(self, cmd: str, timeout=2) -> tuple[ReplyStatus, str]:
        """
        Sends a command and blocks until 'ok' or 'error' is received.
        :param cmd: The command to send.
        :param timeout: Maximum time to wait for response.
        :return: Tuple containing Status enum (OK | ERROR | TIMEOUT), and response lines.
        """
        if self._actor is None:
            return self.ReplyStatus.ERROR, 'Serial actor not running'

        request = _CommandRequest(cmd=(cmd.strip() + "\n"), timeout_s=timeout)
        if self.command_msg_callback:
            self.command_msg_callback(request.cmd, None, '')

        self._actor.enqueue(request)

        # Includes reconnect budget in case the actor is re-establishing the link.
        wait_s = timeout + max(self.reconnect_timeout, 0.5) + 0.5
        if not request.done.wait(timeout=wait_s):
            return self.ReplyStatus.TIMEOUT, request.response_string

        status = request.response_status or self.ReplyStatus.ERROR
        return status, request.response_string

    def close(self):
        """Closes the serial port."""
        if self._actor is not None:
            self._actor.stop()
            self._actor = None
        if self.serial and self.serial.is_open:
            self.serial.close()

# --- OpenMicroStageInterface ------------------------------------------------------------------------------------------

class OpenMicroStageInterface:
    # Mapping log levels to colors
    LOG_COLORS = {
        SerialInterface.LogLevel.DEBUG: f"{Fore.WHITE}{Style.DIM}",
        SerialInterface.LogLevel.INFO: Style.RESET_ALL,
        SerialInterface.LogLevel.WARNING: Fore.YELLOW,
        SerialInterface.LogLevel.ERROR: Fore.RED,
    }

    def __init__(self, show_communication=True, show_log_messages=True):
        self.serial: SerialInterface | None = None
        self.workspace_transform = np.eye(4)
        self.workspace_transform_inv = np.linalg.inv(self.workspace_transform)
        self.show_communication = show_communication
        self.show_log_messages = show_log_messages
        self.disable_message_callbacks = False

    def connect(self, port: str, baud_rate: int = 921600):
        def version_to_str(v):
            return f"v{v[0]}.{v[1]}.{v[2]}"

        if self.serial is not None:
            self.disconnect()
        self.serial = SerialInterface(port, baud_rate,
                                      log_msg_callback=self.log_msg_callback,
                                      command_msg_callback=self.command_msg_callback,
                                      unsolicited_msg_callback=self.unsolicited_msg_callback)

        self.disable_message_callbacks = True
        fw_version = self.read_firmware_version()
        min_fw_version = (1, 0, 1)
        print(f"{Fore.MAGENTA}Firmware version: {version_to_str(fw_version)}{Style.RESET_ALL}")
        if fw_version < min_fw_version:
            print(
                f"{Fore.MAGENTA}Firmware version {version_to_str(fw_version)} incompatible. "
                f"At least {version_to_str(min_fw_version)} required{Style.RESET_ALL}"
            )
            self.serial = None
        print('')
        self.disable_message_callbacks = False

    def disconnect(self):
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def is_connected(self):
        return self.serial is not None

    def log_msg_callback(self, log_level, msg):
        if not self.show_log_messages or self.disable_message_callbacks:
            return

        color = OpenMicroStageInterface.LOG_COLORS.get(log_level, Fore.WHITE)
        if log_level not in [SerialInterface.LogLevel.INFO, SerialInterface.LogLevel.DEBUG]:
            print(f"{color}[{log_level.name}] {msg}{Style.RESET_ALL}")
        else:
            print(f"{color}{msg}{Style.RESET_ALL}")

    def command_msg_callback(self, msg, reply_status: SerialInterface.ReplyStatus | None, error_msg: str):
        if not self.show_communication or self.disable_message_callbacks:
            return

        if reply_status is not None:
            if msg:
                msg = '\n'.join('> ' + line for line in msg.splitlines())
                print(f"{msg.rstrip()}")
            if error_msg:
                print(f"{Style.BRIGHT}{str(reply_status.name)}:{Style.RESET_ALL} {error_msg}\n")
            else:
                print(f"{Style.BRIGHT}{str(reply_status.name)} {Style.RESET_ALL}\n")
        else:
            print(f"{Fore.GREEN}{Style.BRIGHT}{msg.rstrip()}{Style.RESET_ALL}")

    def unsolicited_msg_callback(self, msg):
        print(f"{Fore.CYAN}{msg}{Style.RESET_ALL}")
        pass

    def set_workspace_transform(self, transform):
        self.workspace_transform = transform
        self.workspace_transform_inv = np.linalg.inv(self.workspace_transform)

    def get_workspace_transform(self):
        return self.workspace_transform

    def _serial_or_raise(self) -> SerialInterface:
        if self.serial is None:
            raise RuntimeError("Serial interface is not connected")
        return self.serial

    def read_firmware_version(self):
        ok, response = self._serial_or_raise().send_command("M58")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return 0, 0, 0

        version_match = re.match(r'v(\d+)\.(\d+)\.(\d+)', response)
        if version_match is None:
            return 0, 0, 0

        major, minor, patch = map(int, version_match.groups())
        return major, minor, patch

    def home(self, axis_list=None):
        """
        Homes one or more axes on the device
        :param axis_list: Optional list of axis indices to home. If None, all axes are homed.
        :return: The status of the command (e.g. OK, ERROR, TIMEOUT).
        """
        cmd = 'G28'
        axis_chars = ['A', 'B', 'C', 'D', 'E', 'F']
        if axis_list is None:
            axis_list = list(range(len(axis_chars)))

        for axis_idx in axis_list:
            if 0 > axis_idx >= len(axis_chars):
                raise ValueError('Axis index out of range')
            cmd += ' '+axis_chars[axis_idx]

        res, msg = self._serial_or_raise().send_command(cmd + "\n", 10)
        return res

    def calibrate_joint(self, joint_index: int, save_result: bool):
        """
        Calibrates the given joint and returns the measured data as three lists containing:
            data[0]: list of motor angles
            data[1]: list of electric field angles
            data[2]: list of raw encoder counts
        :param joint_index:
        :param save_result:
        :return:
        """
        cmd = f"M56 J{joint_index} P"
        if save_result:
            cmd += ' S'
        res, msg = self._serial_or_raise().send_command(cmd, 30)

        calibration_data = self._parse_table_data(msg, 3)
        return res, calibration_data

    def move_to(self, x, y, z, f, move_immediately=False, blocking=True, timeout=1):
        """
        Moves the stage to an absolute position with a specified feed rate.
        :param x: Target X position (in workspace coordinates).
        :param y: Target Y position (in workspace coordinates).
        :param z: Target Z position (in workspace coordinates).
        :param f: Feed rate in mm/s.
        :param move_immediately: If True, execution starts without buffering delay.
        :param blocking: If True, waits and retries if the device is busy. If False, returns immediately on 'BUSY'.
        :param timeout: Timeout in seconds for each command attempt.
        :return: Status of the move command (e.g. OK, ERROR, BUSY, TIMEOUT).
        """
        # Convert to homogeneous vector
        transformed = self.workspace_transform @ np.array([x, y, z, 1.0])
        x_t, y_t, z_t = transformed[:3] / transformed[3]

        cmd = f"G0 X{x_t:.6f} Y{y_t:.6f} Z{z_t:.6f} F{f:.3f}"
        if move_immediately:
            cmd += " I"

        # resend messages if queue is full
        while True:
            res, msg = self._serial_or_raise().send_command(cmd + "\n", timeout=timeout)
            if res != SerialInterface.ReplyStatus.BUSY or not blocking:
                return res

    def dwell(self, time_s, blocking, timeout=1):
        cmd = f"G4 S{time_s:.6f}\n"
        # resend messages if queue is full
        while True:
            res, msg = self._serial_or_raise().send_command(cmd + "\n", timeout=timeout)
            if res != SerialInterface.ReplyStatus.BUSY or not blocking:
                return res

    def set_max_acceleration(self, linear_accel, angular_accel):
        linear_accel = max(linear_accel, 0.01)
        angular_accel = max(angular_accel, 0.01)
        cmd = f"M204 L{linear_accel:.6f} A{angular_accel:.6f}\n"
        res, msg = self._serial_or_raise().send_command(cmd)
        return res

    def wait_for_stop(self, polling_interval_ms=10, disable_callbacks=True):
        disable_message_callbacks_prev = self.disable_message_callbacks
        if disable_callbacks:
            self.disable_message_callbacks = True

        while True:
            res, msg = self._serial_or_raise().send_command("M53\n")
            if res != SerialInterface.ReplyStatus.OK:
                return res
            if msg.strip() == "1":
                self.disable_message_callbacks = disable_message_callbacks_prev
                return SerialInterface.ReplyStatus.OK
            time.sleep(polling_interval_ms*0.001)

        self.disable_message_callbacks = disable_message_callbacks_prev

    def read_current_position(self, apply_inv_workspace_transform):
        """
        Reads the current position of the dives EXCLUDING the workspace transform.
        If you want to use the result with a
        """
        ok, response = self._serial_or_raise().send_command("M50")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return None, None, None

        # Match values with NO space between axis letter and number
        match = re.search(
            r"X([-+]?\d*\.?\d+)\s*Y([-+]?\d*\.?\d+)\s*Z([-+]?\d*\.?\d+)",
            response
        )
        if not match:
            raise ValueError(f"Invalid format: {response}")

        x, y, z = match.groups()
        if apply_inv_workspace_transform:
            transformed = self.workspace_transform_inv @ np.array([float(x), float(y), float(z), 1.0])
            x, y, z = transformed[:3] / transformed[3]

        return float(x), float(y), float(z)

    def read_encoder_angles(self):
        ok, response = self._serial_or_raise().send_command("M51")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return []
        return []

    def read_device_state_info(self):
        res, msg = self._serial_or_raise().send_command("M57")
        return res

    def set_servo_parameter(self, pos_kp=150, pos_ki=50000, vel_kp=0.2, vel_ki=100, vel_filter_tc=0.0025):
        cmd = f"M55 A{pos_kp:.6f} B{pos_ki:.6f} C{vel_kp:.6f} D{vel_ki:.6f} F{vel_filter_tc:.6f}"
        res, msg = self._serial_or_raise().send_command(cmd)
        return res

    def enable_motors(self, enable):
        cmd = "M17" if enable else "M18"
        res, msg = self._serial_or_raise().send_command(cmd, timeout=5)
        return res

    def set_pose(self, x, y, z):
        # Convert to homogeneous vector
        transformed = self.workspace_transform @ np.array([x, y, z, 1.0])
        x_t, y_t, z_t = transformed[:3] / transformed[3]

        cmd = f"G24 X{x_t:.6f} Y{y_t:.6f} Z{z_t:.6f}" # TODO: A, B ,C
        res, msg = self._serial_or_raise().send_command(cmd)
        return res

    def send_command(self, cmd: str, timeout_s: float=5):
        res, msg = self._serial_or_raise().send_command(cmd, timeout_s)
        return res, msg

    @staticmethod
    def _parse_table_data(data_string, cols):
        # Parse the data
        data = [[] for _ in range(cols)]

        for line in data_string.strip().splitlines():
            parts = line.strip().split(',')
            if len(parts) != cols:
                continue  # skip malformed lines
            numbers = map(float, parts)
            for i, n in enumerate(numbers):
                data[i].append(n)

        return data

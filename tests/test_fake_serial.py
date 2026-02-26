import os
import sys
import unittest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SOURCE_ROOT = os.path.join(PROJECT_ROOT, "source")
if SOURCE_ROOT not in sys.path:
    sys.path.insert(0, SOURCE_ROOT)

# Imported after local path injection for test-only module discovery.
from hardware.mocks import FakeSerial  # noqa: E402


def _drain_lines(fake: FakeSerial) -> list[str]:
    lines = []
    buf = ""
    while fake.in_waiting:
        ch = fake.read(1).decode("ascii", errors="ignore")
        if ch in ("\n", "\r"):
            if buf:
                lines.append(buf)
                buf = ""
        else:
            buf += ch
    if buf:
        lines.append(buf)
    return lines


def _send(fake: FakeSerial, cmd: str) -> list[str]:
    fake.write((cmd + "\n").encode("ascii"))
    return _drain_lines(fake)


class TestFakeSerial(unittest.TestCase):
    def setUp(self):
        self.fake = FakeSerial()

    def test_m58_returns_version_and_ok(self):
        lines = _send(self.fake, "M58")
        self.assertEqual(lines, ["v1.0.2", "ok"])

    def test_m50_returns_default_position(self):
        lines = _send(self.fake, "M50")
        self.assertEqual(lines, ["X0.000000 Y0.000000 Z0.000000", "ok"])

    def test_m50_reflects_motion_position_after_g0(self):
        self.assertEqual(_send(self.fake, "G0 X1.2 Y-3.4 Z5.6 F10"), ["ok"])
        self.assertEqual(
            _send(self.fake, "M50"),
            ["X1.200000 Y-3.400000 Z5.600000", "ok"],
        )

    def test_m53_reports_moving_then_stopped(self):
        _send(self.fake, "G0 X1 Y2 Z3 F10")
        self.assertEqual(_send(self.fake, "M53"), ["0", "ok"])
        self.assertEqual(_send(self.fake, "M53"), ["1", "ok"])

    def test_m51_returns_encoder_payload_and_ok(self):
        lines = _send(self.fake, "M51")
        self.assertEqual(lines, ["A0.000000 B0.000000 C0.000000", "ok"])

    def test_m56_returns_three_row_table_and_ok(self):
        lines = _send(self.fake, "M56 J0 P")
        self.assertEqual(
            lines,
            [
                "0.0,0.0,100",
                "45.0,43.0,200",
                "90.0,87.0,300",
                "ok",
            ],
        )

    def test_m57_returns_state_and_ok(self):
        lines = _send(self.fake, "M57")
        self.assertEqual(lines, ["state:idle", "ok"])

    def test_simple_ack_commands_return_ok(self):
        for cmd in ["M55 A1 B2 C3 D4 F5", "M17", "M18", "M204 L1.0 A2.0", "G4 S0.25"]:
            with self.subTest(cmd=cmd):
                self.assertEqual(_send(self.fake, cmd), ["ok"])

    def test_g24_sets_pose_and_m50_reflects_it(self):
        self.assertEqual(_send(self.fake, "G24 X9 Y8 Z7"), ["ok"])
        self.assertEqual(_send(self.fake, "M50"), ["X9.000000 Y8.000000 Z7.000000", "ok"])

    def test_g28_homes_position(self):
        _send(self.fake, "G24 X9 Y8 Z7")
        self.assertEqual(_send(self.fake, "G28 A B C"), ["ok"])
        self.assertEqual(_send(self.fake, "M50"), ["X0.000000 Y0.000000 Z0.000000", "ok"])

    def test_unknown_command_returns_error(self):
        lines = _send(self.fake, "M999")
        self.assertEqual(lines, ["error:unsupported"])


if __name__ == "__main__":
    unittest.main()

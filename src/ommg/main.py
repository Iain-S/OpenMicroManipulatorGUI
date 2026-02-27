# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------
# ruff: noqa: I001, E402

import argparse
import asyncio
import os

# Scale-related defaults. We preserve explicit user/system values.
SCALING_ENV_DEFAULTS = {
    "QT_SCALE_FACTOR": "1",
    "QT_AUTO_SCREEN_SCALE_FACTOR": "0",
    "GDK_SCALE": "1",
    "GDK_DPI_SCALE": "1",
}


def _configure_scaling_env() -> None:
    for key, default in SCALING_ENV_DEFAULTS.items():
        existing = os.environ.get(key)
        if existing is None:
            os.environ[key] = default
        elif existing != default:
            print(
                f"[main] Warning: preserving existing {key}={existing!r}; "
                f"default would be {default!r}"
            )


_configure_scaling_env()

import cv2
from PySide6.QtWidgets import QApplication
from qasync import QEventLoop

from ommg.hardware.camera_mock_robot import MockRobotViewportCamera
from ommg.hardware.camera_opencv import OpenCVCamera
from ommg.hardware.open_micro_stage_api import OpenMicroStageInterface
from ommg.mainwindow import DeviceControlMainWindow

EXPOSURE_TIME_US = 16_000
MOCK_VIEWPORT_SCALE_DEFAULT = 0.45
MOCK_XY_MM_TO_PX_DEFAULT = 140.0
MOCK_Z_MM_TO_SCALE_DEFAULT = 0.05


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Open Micro-Manipulator GUI")
    parser.add_argument("--port", default="mock", help="Serial port (e.g. /dev/ttyACM0, COM1, mock)")
    parser.add_argument("--baud-rate", type=int, default=921600, help="Serial baud rate")
    parser.add_argument("--camera-backend", choices=("opencv", "basler"), default="opencv")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--exposure-us", type=float, default=EXPOSURE_TIME_US, help="Camera exposure in microseconds")
    parser.add_argument(
        "--use-mock-camera",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable mock robot viewport camera wrapper",
    )
    parser.add_argument(
        "--mock-viewport-scale",
        type=float,
        default=None,
        help="Mock camera viewport crop scale (requires --use-mock-camera)",
    )
    parser.add_argument(
        "--mock-xy-mm-to-px",
        type=float,
        default=None,
        help="Mock camera XY mm-to-pixel factor (requires --use-mock-camera)",
    )
    parser.add_argument(
        "--mock-z-mm-to-scale",
        type=float,
        default=None,
        help="Mock camera Z-to-zoom factor (requires --use-mock-camera)",
    )
    parser.add_argument("--pixel-per-mm", type=float, default=2000.0)
    parser.add_argument(
        "--show-communication",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Print sent commands and command responses",
    )
    parser.add_argument(
        "--show-log-messages",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Print device log messages",
    )
    return parser


def parse_args() -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args()

    incompatible_mock_flags: list[str] = []
    if not args.use_mock_camera:
        if args.mock_viewport_scale is not None:
            incompatible_mock_flags.append("--mock-viewport-scale")
        if args.mock_xy_mm_to_px is not None:
            incompatible_mock_flags.append("--mock-xy-mm-to-px")
        if args.mock_z_mm_to_scale is not None:
            incompatible_mock_flags.append("--mock-z-mm-to-scale")

    if incompatible_mock_flags:
        parser.error(
            "incompatible arguments: "
            f"{', '.join(incompatible_mock_flags)} cannot be used with --no-use-mock-camera"
        )

    return args


def _to_vis_image(frame):
    if len(frame.shape) == 2 or frame.shape[2] == 1:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame.copy()


async def _run_camera_loop(camera, gui, pixel_per_mm: float):
    camera.start_grabbing(single_grab=False)
    try:
        while gui.isVisible() and camera.is_connected():
            frame = camera.grab_one(timeout_ms=100)
            if frame is None:
                await asyncio.sleep(0.005)
                continue

            vis_img = _to_vis_image(frame)
            gui.update_controller(frame, vis_img, pixel_per_mm=pixel_per_mm)
            await asyncio.sleep(0)
    finally:
        camera.stop_grabbing()


def main():
    args = parse_args()

    oms = OpenMicroStageInterface(
        show_communication=args.show_communication,
        show_log_messages=args.show_log_messages,
    )
    oms.connect(args.port, args.baud_rate)

    if args.camera_backend == "basler":
        from ommg.hardware.camera_basler import BaslerCamera

        base_camera = BaslerCamera()
    else:
        base_camera = OpenCVCamera(camera_index=args.camera_index)

    base_camera.set_exposure_time(args.exposure_us)
    if args.use_mock_camera:
        camera = MockRobotViewportCamera(
            source_camera=base_camera,
            oms=oms,
            viewport_scale=args.mock_viewport_scale
            if args.mock_viewport_scale is not None else MOCK_VIEWPORT_SCALE_DEFAULT,
            xy_mm_to_px=args.mock_xy_mm_to_px
            if args.mock_xy_mm_to_px is not None else MOCK_XY_MM_TO_PX_DEFAULT,
            z_mm_to_scale=args.mock_z_mm_to_scale
            if args.mock_z_mm_to_scale is not None else MOCK_Z_MM_TO_SCALE_DEFAULT,
        )
    else:
        camera = base_camera

    app = QApplication()
    gui = DeviceControlMainWindow(oms, camera)
    gui.show()

    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)

    app.aboutToQuit.connect(loop.stop)

    if camera.is_connected():
        loop.create_task(_run_camera_loop(camera, gui, pixel_per_mm=args.pixel_per_mm))

    with loop:
        loop.run_forever()

if __name__ == "__main__":
    main()

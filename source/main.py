# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import argparse
import os

# Disable scaling
os.environ['QT_SCALE_FACTOR'] = '1'
os.environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
os.environ['GDK_SCALE'] = '1'
os.environ['GDK_DPI_SCALE'] = '1'

import cv2
from hardware.camera_mock_robot import MockRobotViewportCamera
from hardware.camera_opencv import OpenCVCamera
from hardware.open_micro_stage_api import OpenMicroStageInterface
from mainwindow import DeviceControlMainWindow
from PySide6.QtWidgets import QApplication

EXPOSURE_TIME_US = 16_000

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
    parser.add_argument("--mock-viewport-scale", type=float, default=0.45)
    parser.add_argument("--mock-xy-mm-to-px", type=float, default=140.0)
    parser.add_argument("--mock-z-mm-to-scale", type=float, default=0.05)
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


def main():
    args = build_parser().parse_args()

    # --- configuration -------------------------------------------------------
    # create interface and connect
    oms = OpenMicroStageInterface(
        show_communication=args.show_communication,
        show_log_messages=args.show_log_messages,
    )
    oms.connect(args.port, args.baud_rate)

    # Setup camera
    if args.camera_backend == "basler":
        from hardware.camera_basler import BaslerCamera

        base_camera = BaslerCamera()
    else:
        base_camera = OpenCVCamera(camera_index=args.camera_index)

    base_camera.set_exposure_time(args.exposure_us)
    camera = base_camera
    if args.use_mock_camera:
        camera = MockRobotViewportCamera(
            source_camera=base_camera,
            oms=oms,
            viewport_scale=args.mock_viewport_scale,
            xy_mm_to_px=args.mock_xy_mm_to_px,
            z_mm_to_scale=args.mock_z_mm_to_scale,
        )

    # ------------------------------------------------------------------------

    # create the Qt app and GUI
    app = QApplication()
    gui = DeviceControlMainWindow(oms, camera)
    gui.show()

    def process_frame(frame):
        #frame = cv2.flip(frame, 1)

        # Convert grayscale to BGR if needed
        if len(frame.shape) == 2 or frame.shape[2] == 1:
            vis_img = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            vis_img = frame.copy()

        gui.update_controller(frame, vis_img, pixel_per_mm=args.pixel_per_mm)

        app.processEvents()
        a = gui.isVisible()
        return a

    if camera.is_connected():
        # Run the camera loop (which also runs qt event loop)
        camera.grab_loop(callback=process_frame)
    else:
        # Start the Qt event loop
        app.exec()

if __name__ == "__main__":
    main()

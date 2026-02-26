# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

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

def main():

    # --- change configuration here ----------------------------------------------
    use_mock_robot_camera = True

    # create interface and connect
    oms = OpenMicroStageInterface(show_communication=False, show_log_messages=True)
    # oms.connect('/dev/ttyACM0')       # on linux
    oms.connect('mock')       # on linux
    # oms.connect('COM1')             # on windows

    # Setup camera
    # camera = BaslerCamera()
    base_camera = OpenCVCamera(camera_index=0)
    base_camera.set_exposure_time(EXPOSURE_TIME_US)
    camera = base_camera
    if use_mock_robot_camera:
        camera = MockRobotViewportCamera(
            source_camera=base_camera,
            oms=oms,
            viewport_scale=0.45,   # visible central rectangle as fraction of source frame
            xy_mm_to_px=140.0,     # how strongly stage XY moves viewport center
            z_mm_to_scale=0.05,    # Z changes viewport zoom
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

        gui.update_controller(frame, vis_img, pixel_per_mm=2000.0)

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

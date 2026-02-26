# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import time

import cv2
import numpy as np

from ommg.hardware.abstract_camera import AbstractCamera


class MockRobotViewportCamera(AbstractCamera):
    """
    Wraps a real camera and returns a transformed "virtual camera" image.
    The output is a cropped viewport from the source frame, where viewport
    center follows the stage XY position and viewport size can vary with Z.
    """

    def __init__(self, source_camera: AbstractCamera, oms,
                 viewport_scale: float = 0.5,
                 xy_mm_to_px: float = 100.0,
                 z_mm_to_scale: float = 0.03):
        self.source_camera = source_camera
        self.oms = oms
        self.viewport_scale = viewport_scale
        self.xy_mm_to_px = xy_mm_to_px
        self.z_mm_to_scale = z_mm_to_scale
        self.grabbing = False

        self._last_pose = np.zeros(3, dtype=np.float32)

    def set_mock_view_params(self, viewport_scale=None, xy_mm_to_px=None, z_mm_to_scale=None):
        if viewport_scale is not None:
            self.viewport_scale = float(np.clip(viewport_scale, 0.25, 0.9))
        if xy_mm_to_px is not None:
            self.xy_mm_to_px = float(np.clip(xy_mm_to_px, 1.0, 2000.0))
        if z_mm_to_scale is not None:
            self.z_mm_to_scale = float(np.clip(z_mm_to_scale, 0.0, 1.0))

    def get_mock_view_params(self):
        return {
            "viewport_scale": self.viewport_scale,
            "xy_mm_to_px": self.xy_mm_to_px,
            "z_mm_to_scale": self.z_mm_to_scale,
        }

    def is_connected(self):
        return self.source_camera.is_connected()

    def get_exposure_time_range(self):
        return self.source_camera.get_exposure_time_range()

    def set_exposure_time(self, exposure_time_us):
        self.source_camera.set_exposure_time(exposure_time_us)

    def start_grabbing(self, single_grab=True):
        self.grabbing = True
        self.source_camera.start_grabbing(single_grab=single_grab)

    def stop_grabbing(self):
        self.grabbing = False
        self.source_camera.stop_grabbing()

    def grab_single_triggered(self, timeout_ms=1000):
        frame = self.source_camera.grab_single_triggered(timeout_ms)
        return self._apply_mock_motion(frame)

    def grab_one(self, timeout_ms=5000):
        frame = self.source_camera.grab_one(timeout_ms)
        return self._apply_mock_motion(frame)

    def grab_loop(self, callback, timeout_ms=5000):
        if not self.is_connected():
            return

        self.start_grabbing(single_grab=False)
        try:
            while self.grabbing:
                frame = self.source_camera.grab_one(timeout_ms)
                if frame is None:
                    time.sleep(0.01)
                    continue

                frame = self._apply_mock_motion(frame)

                try:
                    if callback(frame) is False:
                        break
                except Exception as e:
                    print(f"Error in callback: {e}")
        finally:
            self.stop_grabbing()

    def _read_stage_pose(self):
        if self.oms is None or not self.oms.is_connected():
            return self._last_pose

        pose = self.oms.read_current_position(True)
        if pose is None or pose[0] is None:
            return self._last_pose

        self._last_pose[:] = pose
        return self._last_pose

    def _apply_mock_motion(self, frame):
        if frame is None:
            return None

        h, w = frame.shape[:2]
        if h <= 2 or w <= 2:
            return frame

        x_mm, y_mm, z_mm = self._read_stage_pose()

        dynamic_scale = self.viewport_scale - float(z_mm) * self.z_mm_to_scale
        dynamic_scale = max(0.25, min(0.9, dynamic_scale))

        crop_w = max(20, int(w * dynamic_scale))
        crop_h = max(20, int(h * dynamic_scale))

        cx = int(w * 0.5 + float(x_mm) * self.xy_mm_to_px)
        cy = int(h * 0.5 - float(y_mm) * self.xy_mm_to_px)
        cx = int(np.clip(cx, crop_w // 2, w - crop_w // 2))
        cy = int(np.clip(cy, crop_h // 2, h - crop_h // 2))

        x0 = cx - crop_w // 2
        y0 = cy - crop_h // 2
        x1 = x0 + crop_w
        y1 = y0 + crop_h

        cropped = frame[y0:y1, x0:x1]
        if cropped.size == 0:
            return frame

        return cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)

#!/usr/bin/env python3
"""PyQt5 GUI for tuning the lateral and forward PID controllers.

Replaces aruco_tracker.py + lateral_tuner + forward_tuner + key_pub.py
during tuning sessions. Subscribes to /master/telemetry, publishes
/master/commands. Detects the ArUco marker internally from RTSP so it can
use the true forward range (tvec[2]) instead of the buggy /aruco/error.y.

The file is one module split into seven sections (A-G); jump to the
banner comment for the part you want to debug.
"""

import os
import sys
import threading
import time as _time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2  # imported before PyQt5 on purpose; see env-var fix below
import numpy as np

# opencv-python's wheel bundles its own Qt5 plugins under cv2/qt/plugins and
# pins QT_QPA_PLATFORM_PLUGIN_PATH at import time. PyQt5 then loads from the
# wrong place and aborts with "Could not load the Qt platform plugin xcb".
# Drop the override so Qt falls back to PyQt5's bundled plugin directory.
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

# Gst.init() initialises GLib's threading context, which conflicts with
# FastDDS's boost::interprocess shared-memory setup and causes a segfault
# inside rclpy Node.__init__. CycloneDDS uses plain UDP transport with no
# shared-memory dependency, so the two libraries coexist safely.
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

# GStreamer via PyGObject. We use this for capture instead of
# cv2.VideoCapture(..., CAP_GSTREAMER) because the pip opencv-python wheel
# is built without GStreamer support. Pulling frames through gi gives us a
# direct, low-latency path mirroring the working `gst-launch` pipeline.
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from custom_msgs.msg import Commands, Telemetry

# rclpy.init MUST happen before Gst.init and before PyQt5 loads.
# GStreamer's init grabs boost::interprocess shared memory; if it runs
# before FastDDS the DDS node creation crashes with interprocess_exception.
rclpy.init(args=sys.argv)
Gst.init(None)


class TunerNode(Node):
    """Thin ROS2 bridge. Telemetry callback bridges to Qt via a callback
    set by ControlEngine; publish() is called from the Qt thread."""

    def __init__(self):
        super().__init__("pid_tuner_gui")
        self.declare_parameter("target_id", 28)
        self.declare_parameter("rtsp_url", "rtsp://192.168.2.6:2000/image_rtsp")
        self.target_id = int(self.get_parameter("target_id").value)
        self.rtsp_url = str(self.get_parameter("rtsp_url").value)

        self._cmd_pub = self.create_publisher(Commands, "/master/commands", 1)
        self._tel_sub = self.create_subscription(
            Telemetry, "/master/telemetry", self._on_telemetry, 1)
        self._cmd_sub = self.create_subscription(
            Commands, "/master/commands", self._on_commands, 1)
        self.on_heading = None  # set by ControlEngine

        # Ring buffer written by ROS spin thread, read by Qt plot timer.
        # deque append/read is GIL-safe for single producer + single consumer.
        self.lateral_plot_buf: deque = deque(maxlen=1000)  # 10 s × 100 Hz

    def _on_telemetry(self, msg: Telemetry):
        if self.on_heading is not None:
            self.on_heading(int(msg.heading))

    def _on_commands(self, msg: Commands):
        self.lateral_plot_buf.append((_time.monotonic(), float(msg.lateral)))

    def publish_commands(self, cmd: Commands):
        self._cmd_pub.publish(cmd)


_NODE = TunerNode()  # created before PyQt5 to avoid segfault

from PyQt5.QtCore import (QObject, Qt, QThread, QTimer, pyqtSignal, pyqtSlot)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QDoubleSpinBox, QFrame,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QPlainTextEdit, QPushButton, QRadioButton, QSizePolicy, QSlider,
    QSpinBox, QVBoxLayout, QWidget,
)
import pyqtgraph as pg


MARKER_LENGTH = 0.15  # metres — default, overridable from GUI
PWM_NEUTRAL = 1500
PWM_MIN = 1100
PWM_MAX = 1900
CONTROL_HZ = 100
DISPLAY_HZ = 15
DETECT_HZ = 30                 # cap detection rate to keep CPU sane
DETECT_MAX_WIDTH = 960         # downscale wider frames before detection
PLOT_WINDOW_S = 10.0
STALE_POSE_S = 0.5
LOOP_STALL_S = 0.2
LOG_MAX_LINES = 500


# ============================================================
# SECTION A — ArUco detection (ported from aruco_tracker.py)
# ============================================================

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
if hasattr(cv2.aruco, "ArucoDetector"):
    _ARUCO_PARAMS = cv2.aruco.DetectorParameters()
    _DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, _ARUCO_PARAMS)

    def _detect_markers(frame):
        return _DETECTOR.detectMarkers(frame)
else:
    _ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

    def _detect_markers(frame):
        return cv2.aruco.detectMarkers(frame, ARUCO_DICT, parameters=_ARUCO_PARAMS)




@dataclass
class MarkerPose:
    """3-axis pose in camera optical frame: x=right, y=down, z=forward (m)."""
    x: float
    y: float
    z: float


class ArucoDetector:
    def __init__(self, calib_path: str, target_id: int,
                 marker_length: float = MARKER_LENGTH):
        if not os.path.exists(calib_path):
            raise FileNotFoundError(calib_path)
        calib = np.load(calib_path)
        self.camera_matrix = calib["mtx"].astype(np.float64)
        self.dist_coeffs = calib["dist"].astype(np.float64)
        self.target_id = int(target_id)
        self._marker_length = marker_length
        self._obj_points = self._make_obj_points(marker_length)

    @staticmethod
    def _make_obj_points(length: float) -> np.ndarray:
        hl = length / 2.0
        return np.array([
            [-hl,  hl, 0],
            [ hl,  hl, 0],
            [ hl, -hl, 0],
            [-hl, -hl, 0],
        ], dtype=np.float32)

    def set_target_id(self, target_id: int):
        self.target_id = int(target_id)

    def set_marker_length(self, length_m: float):
        self._marker_length = max(0.01, float(length_m))
        self._obj_points = self._make_obj_points(self._marker_length)

    def process_frame(
        self, frame_bgr: np.ndarray
    ) -> Tuple[np.ndarray, Optional[MarkerPose], list]:
        """Run underwater enhancement, detect target marker, return
        (annotated frame, pose or None, list of all detected IDs)."""
        # CLAHE on L channel — boosts contrast for underwater visibility.
        # Bilateral filter and sharpen were dropped: ArUco's internal
        # thresholding doesn't need them, and bilateral was the bulk of
        # detection time on every frame.
        lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        l = clahe.apply(l)
        frame = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)

        corners, ids, _ = _detect_markers(frame)

        h, w = frame.shape[:2]
        cx_pix, cy_pix = w // 2, h // 2
        pose: Optional[MarkerPose] = None
        detected_ids: list = ([] if ids is None
                              else [int(x) for x in ids.flatten()])

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i, marker_id in enumerate(ids.flatten()):
                if int(marker_id) != self.target_id:
                    continue
                obj_pts = self._obj_points
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts, corners[i], self.camera_matrix,
                    self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if not ok:
                    continue
                cv2.solvePnPRefineLM(
                    obj_pts, corners[i], self.camera_matrix,
                    self.dist_coeffs, rvec, tvec,
                )
                tvec = tvec.flatten()
                pose = MarkerPose(float(tvec[0]), float(tvec[1]), float(tvec[2]))

                cv2.drawFrameAxes(
                    frame, self.camera_matrix, self.dist_coeffs,
                    rvec, tvec, self._marker_length * 0.75,
                )
                label = f"ID:{marker_id} x:{tvec[0]:+.2f} y:{tvec[1]:+.2f} z:{tvec[2]:+.2f}"
                pt = tuple(corners[i][0][0].astype(int))
                cv2.putText(frame, label, pt, cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

                mc = (int(corners[i][0][:, 0].mean()),
                      int(corners[i][0][:, 1].mean()))
                cv2.line(frame, mc, (cx_pix, cy_pix), (0, 255, 255), 2)
                cv2.circle(frame, mc, 5, (0, 255, 255), -1)
                break

        # Full-frame red crosshair through image centre
        cv2.line(frame, (0, cy_pix), (w, cy_pix), (0, 0, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (cx_pix, 0), (cx_pix, h), (0, 0, 255), 1, cv2.LINE_AA)

        # Top-left HUD: marker position in camera frame (m), or ?? if missing
        if pose is not None:
            hud = (f"x: {pose.x:+.3f} m   "
                   f"y: {pose.y:+.3f} m   "
                   f"z: {pose.z:.3f} m")
        else:
            hud = "x: ??   y: ??   z: ??"
        cv2.putText(frame, hud, (12, 32), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, hud, (12, 32), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 1, cv2.LINE_AA)
        return frame, pose, detected_ids


# ============================================================
# SECTION B — PID controller (Python port of control_utils.hpp)
# ============================================================

class PIDController:
    """Faithful port of PID_Controller from control_utils.hpp.

    Matches the C++ trapezoidal integration (which uses time intervals
    rather than cumulative time — see control_utils.hpp:160-161). At a
    fixed 100 Hz tick the integrand factor (dt[i] - dt[i-1]) ~= 0, so Ki
    barely contributes; this is a quirk inherited from the C++ tuners,
    preserved here so gains transfer between the GUI and the legacy
    nodes.
    """

    MAX_HISTORY = 100
    SAFE_PWM = 400  # output clamp magnitude, applied around 1500

    def __init__(self, name: str, kp: float = 0.0, ki: float = 0.0,
                 kd: float = 0.0, base_offset: float = float(PWM_NEUTRAL)):
        self.name = name
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.base_offset = base_offset
        self._times = deque(maxlen=self.MAX_HISTORY)
        self._errors = deque(maxlen=self.MAX_HISTORY)
        self._prev_d = 0.0

    def reset(self):
        self._times.clear()
        self._errors.clear()
        self._prev_d = 0.0

    def control(self, error: float, dt: float) -> int:
        self._times.append(dt)
        self._errors.append(error)

        p_term = self.kp * error
        i_term = self.ki * self._integrate()
        if len(self._errors) >= 2 and self._errors[-1] != self._errors[-2]:
            d_term = self.kd * (self._errors[-1] - self._errors[-2])
        else:
            d_term = self._prev_d
        self._prev_d = d_term

        raw = self.base_offset + p_term + i_term + d_term
        clamped = max(PWM_NEUTRAL - self.SAFE_PWM,
                      min(PWM_NEUTRAL + self.SAFE_PWM, raw))
        return int(clamped)

    def _integrate(self) -> float:
        area = 0.0
        t = self._times
        e = self._errors
        for i in range(1, len(t)):
            area += (t[i] - t[i - 1]) * (e[i] + e[i - 1]) / 2.0
        return area


# ============================================================
# SECTION C — Video grabber thread
# ============================================================

def _build_gst_pipeline(url: str) -> str:
    """GStreamer pipeline string consumed by Gst.parse_launch.

    Mirrors the gst-launch pipeline that's known to work on this network:
      rtspsrc location=URL latency=0 buffer-mode=auto !
        rtph264depay ! h264parse ! avdec_h264 !
        videoconvert ! autovideosink sync=false

    Two changes for in-process ingestion:
      - autovideosink → appsink (so we can pull frames from Python)
      - explicit BGR caps before appsink (so we get the right pixel order)
      - max-buffers=1 + drop=true keeps latency live if the GUI lags
    """
    return (
        f"rtspsrc location={url} latency=0 buffer-mode=auto ! "
        "rtph264depay ! h264parse ! avdec_h264 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=sink emit-signals=false sync=false "
        "max-buffers=1 drop=true"
    )


class GstCapture:
    """Tiny cv2.VideoCapture-shaped wrapper over a GStreamer appsink.

    Exists because the pip `opencv-python` wheel ships without GStreamer
    support, but PyGObject is already in use elsewhere in this repo (see
    publish_rtsp_stream.py). The API matches what VideoGrabber needs:
    isOpened(), read() returning (ok, bgr_ndarray), and release().
    """

    def __init__(self, url: str, open_timeout_s: float = 5.0):
        self._pipeline = Gst.parse_launch(_build_gst_pipeline(url))
        self._sink = self._pipeline.get_by_name("sink")
        self._opened = False
        self._pipeline.set_state(Gst.State.PLAYING)
        # Wait for the pipeline to actually reach PLAYING (rtspsrc needs
        # a few hundred ms to negotiate). If it fails / times out, mark
        # not-opened so the caller can retry.
        ret, state, _pending = self._pipeline.get_state(
            int(open_timeout_s * Gst.SECOND))
        if ret == Gst.StateChangeReturn.SUCCESS and state == Gst.State.PLAYING:
            self._opened = True

    def isOpened(self) -> bool:
        return self._opened

    def read(self, timeout_s: float = 1.0):
        """Block up to timeout_s for the next frame, then drain the appsink
        to return the *most recent* frame available. Returns (ok, frame).

        Why drain: appsink keeps max-buffers=1 with drop=true, so it only
        ever holds one sample — but in the time we spent doing detection,
        upstream may have pushed several frames. Pulling one then immediately
        polling non-blockingly for any newer ones makes sure we never paint
        a stale frame from before the last detection cycle.
        """
        sample = self._sink.emit("try-pull-sample",
                                 int(timeout_s * Gst.SECOND))
        if sample is None:
            return False, None
        # Drain — keep the freshest sample queued behind the first pull.
        while True:
            newer = self._sink.emit("try-pull-sample", 0)
            if newer is None:
                break
            sample = newer

        buf = sample.get_buffer()
        caps = sample.get_caps().get_structure(0)
        w = caps.get_value("width")
        h = caps.get_value("height")
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return False, None
        try:
            # appsink delivers tightly-packed BGR (we set the caps), so a
            # straight reshape works. .copy() because mapinfo.data goes
            # away when we unmap below.
            frame = np.frombuffer(mapinfo.data, dtype=np.uint8) \
                      .reshape((h, w, 3)).copy()
        finally:
            buf.unmap(mapinfo)
        return True, frame

    def release(self):
        self._pipeline.set_state(Gst.State.NULL)
        self._opened = False


class VideoGrabber(QThread):
    """Reads frames from RTSP/file, runs ArUco detection in this worker
    thread, and emits the annotated frame + pose. Detection lives here
    (not on the GUI thread) so the Qt event loop stays responsive even
    when bilateral filter / CLAHE / solvePnP take tens of ms each."""

    # Carries (annotated BGR ndarray, MarkerPose|None)
    frame_ready = pyqtSignal(np.ndarray, object)
    status = pyqtSignal(str)

    def __init__(self, detector: ArucoDetector, parent=None):
        super().__init__(parent)
        self.detector = detector
        self._url = ""
        self._url_lock = threading.Lock()
        self._stop = threading.Event()
        self._last_id_log = 0.0

    def set_url(self, url: str):
        with self._url_lock:
            self._url = url

    def stop(self):
        self._stop.set()

    def run(self):
        cap: Optional[GstCapture] = None
        current_url = ""
        while not self._stop.is_set():
            with self._url_lock:
                desired = self._url

            # URL changed → drop any existing capture so we reopen below
            if desired != current_url:
                if cap is not None:
                    cap.release()
                    cap = None
                current_url = desired

            if not current_url:
                self.msleep(100)
                continue

            # No capture yet (initial open OR retry after failure)
            if cap is None:
                self.status.emit(f"[VIDEO] opening {current_url} (GStreamer)")
                try:
                    cap = GstCapture(current_url)
                except Exception as exc:
                    self.status.emit(
                        f"[VIDEO] GStreamer pipeline build failed: {exc}")
                    cap = None
                if cap is not None and not cap.isOpened():
                    self.status.emit(
                        f"[VIDEO] failed to open {current_url} — "
                        "check the RTSP server is running and the "
                        "rtspsrc/avdec_h264 GStreamer plugins are installed "
                        "— retry in 2s")
                    cap.release()
                    cap = None
                if cap is None:
                    # Sleep in small chunks so a URL change is picked up quickly
                    for _ in range(20):
                        if self._stop.is_set():
                            break
                        with self._url_lock:
                            if self._url != current_url:
                                break
                        self.msleep(100)
                    continue
                self.status.emit("[VIDEO] stream open")
                last_process_t = 0.0

            ok, frame = cap.read()
            if not ok or frame is None:
                self.status.emit("[VIDEO] frame read failed — reopening")
                cap.release()
                cap = None
                self.msleep(500)
                continue

            # Throttle detection to DETECT_HZ — no point processing every
            # 60 fps frame for a tuning tool, halves CPU.
            now = _time.monotonic()
            if now - last_process_t < (1.0 / DETECT_HZ):
                continue
            last_process_t = now

            # Downscale aggressively before detection; ArUco is robust
            # well below the source resolution and bilateral/CLAHE scale
            # superlinearly with pixel count.
            h, w = frame.shape[:2]
            if w > DETECT_MAX_WIDTH:
                scale = DETECT_MAX_WIDTH / float(w)
                frame = cv2.resize(frame,
                                   (int(w * scale), int(h * scale)),
                                   interpolation=cv2.INTER_AREA)

            annotated, pose, detected_ids = self.detector.process_frame(frame)
            self.frame_ready.emit(annotated, pose)

            # Once per second, surface what IDs are in view so the user can
            # pick the right Target ArUco ID without guessing.
            if now - self._last_id_log > 1.0:
                self._last_id_log = now
                if detected_ids:
                    self.status.emit(
                        f"[ARUCO] detected IDs in frame: {detected_ids} "
                        f"(target = {self.detector.target_id})")
                else:
                    self.status.emit("[ARUCO] no markers visible")
        if cap is not None:
            cap.release()


# (TunerNode is defined above, before PyQt5 imports, to avoid segfault)


# ============================================================
# SECTION E — Main control loop (binds detector + PID + ROS)
# ============================================================

class ControlEngine(QObject):
    """Runs PID at CONTROL_HZ on a dedicated background thread.
    Plot samples are buffered and flushed to the GUI at DISPLAY_HZ
    so that the Qt event loop stays free for video display."""

    log = pyqtSignal(str)
    # Batched: list of (t, value) tuples, emitted at DISPLAY_HZ
    error_samples_batch = pyqtSignal(list)
    pwm_samples_batch = pyqtSignal(list)
    centred = pyqtSignal(float)              # elapsed seconds
    centred_status = pyqtSignal(bool, float) # in_band, elapsed

    AXES = ("lateral", "forward")

    def __init__(self, node: TunerNode, detector: ArucoDetector):
        super().__init__()
        self.node = node
        self.detector = detector

        self.lateral_pid = PIDController("lateral", kp=200.0, ki=0.0, kd=0.0)
        self.forward_pid = PIDController("forward", kp=200.0, ki=0.0, kd=0.0)
        self.yaw_pid = PIDController("yaw", kp=3.18, ki=0.01, kd=7.2)

        self.armed = False
        self.axis = "lateral"
        self.yaw_lock_enabled = True
        self.tolerance = 0.05
        self.dwell_s = 1.5

        self._last_pose: Optional[MarkerPose] = None
        self._last_pose_time = 0.0
        self._heading: Optional[int] = None
        self._target_heading: Optional[int] = None
        self._telemetry_received = False

        self._prev_tick = _time.monotonic()
        self._t0 = _time.monotonic()
        self._last_loop_warn = 0.0
        self._in_band_since: Optional[float] = None
        self._centred_already_fired = False

        # Buffers for batched plot emission (filled by _tick thread,
        # drained by _flush_plots on a slow QTimer).
        self._err_buf_lock = threading.Lock()
        self._err_buf: list = []   # [(t, err), ...]
        self._pwm_buf: list = []   # [(t, pwm), ...]

        self.node.on_heading = self._on_heading

        self._stop_event = threading.Event()
        self._tick_thread = threading.Thread(
            target=self._tick_loop, daemon=True, name="pid_tick")
        self._tick_thread.start()

        # Slow QTimer only to flush buffered plot data to GUI.
        self._flush_timer = QTimer()
        self._flush_timer.timeout.connect(self._flush_plots)
        self._flush_timer.start(int(1000 / DISPLAY_HZ))

    # ---- inputs from other components --------------------------------

    def submit_pose(self, pose: Optional[MarkerPose]):
        if pose is not None:
            self._last_pose = pose
            self._last_pose_time = _time.monotonic()

    def _on_heading(self, heading: int):
        self._heading = heading
        self._telemetry_received = True
        if self._target_heading is None:
            self._target_heading = heading
            self.log.emit(f"[YAW] heading locked at {heading} deg")

    # ---- GUI-facing setters ------------------------------------------

    def set_armed(self, armed: bool):
        if armed == self.armed:
            return
        self.armed = armed
        if armed:
            self.log.emit("[ARM] software arm = true")
        else:
            self.log.emit("[ARM] software arm = false (integrators reset)")
            self.lateral_pid.reset()
            self.forward_pid.reset()
            self.yaw_pid.reset()
        self._reset_centre_tracking()

    def set_axis(self, axis: str):
        if axis not in self.AXES:
            return
        if axis == self.axis:
            return
        self.axis = axis
        self.lateral_pid.reset()
        self.forward_pid.reset()
        self._reset_centre_tracking()
        self.log.emit(f"[AXIS] now tuning {axis}")

    def set_yaw_lock(self, enabled: bool):
        self.yaw_lock_enabled = enabled
        if not enabled:
            self.yaw_pid.reset()
        self.log.emit(f"[YAW] heading lock {'ON' if enabled else 'OFF'}")

    def set_tolerance(self, tol: float):
        self.tolerance = max(0.0, float(tol))
        self._reset_centre_tracking()

    def set_dwell(self, dwell: float):
        self.dwell_s = max(0.0, float(dwell))
        self._reset_centre_tracking()

    def set_gain(self, axis: str, name: str, value: float):
        pid = self.lateral_pid if axis == "lateral" else self.forward_pid
        if name == "kp":
            pid.kp = value
        elif name == "ki":
            pid.ki = value
        elif name == "kd":
            pid.kd = value

    # ---- main loop ---------------------------------------------------

    def _active_pid(self) -> PIDController:
        return self.lateral_pid if self.axis == "lateral" else self.forward_pid

    def _axis_error(self) -> Optional[float]:
        if self._last_pose is None:
            return None
        if self.axis == "lateral":
            return self._last_pose.x  # camera X = sideways (m)
        return self._last_pose.z      # camera Z = forward range (m)

    def _yaw_error(self) -> int:
        if self._heading is None or self._target_heading is None:
            return 0
        err = float(self._target_heading - self._heading)
        err = (err + 180.0) % 360.0
        if err < 0:
            err += 360.0
        return int(err - 180.0)

    def _reset_centre_tracking(self):
        self._in_band_since = None
        self._centred_already_fired = False
        self.centred_status.emit(False, 0.0)

    def stop(self):
        """Signal the background PID thread to exit."""
        self._stop_event.set()

    def _tick_loop(self):
        """Runs on a dedicated thread at CONTROL_HZ."""
        interval = 1.0 / CONTROL_HZ
        while not self._stop_event.is_set():
            self._tick()
            self._stop_event.wait(interval)

    def _tick(self):
        now = _time.monotonic()
        dt = now - self._prev_tick
        self._prev_tick = now

        if dt > LOOP_STALL_S and (now - self._last_loop_warn) > 1.0:
            self.log.emit(f"[CRITICAL] loop stalled, gap={dt:.3f}s")
            self._last_loop_warn = now

        pose_age = now - self._last_pose_time if self._last_pose_time else 1e9
        pose_fresh = self._last_pose is not None and pose_age < STALE_POSE_S

        cmd = Commands()
        cmd.mode = "ALT_HOLD"
        cmd.arm = self.armed
        cmd.forward = PWM_NEUTRAL
        cmd.lateral = PWM_NEUTRAL
        cmd.thrust = PWM_NEUTRAL
        cmd.pitch = PWM_NEUTRAL
        cmd.roll = PWM_NEUTRAL
        cmd.yaw = PWM_NEUTRAL
        cmd.servo1 = PWM_NEUTRAL
        cmd.servo2 = PWM_NEUTRAL

        axis_err = self._axis_error()
        axis_pwm: Optional[int] = None

        if self.armed and pose_fresh and axis_err is not None:
            pid = self._active_pid()
            axis_pwm = pid.control(axis_err, dt)
            if self.axis == "lateral":
                cmd.lateral = axis_pwm
            else:
                cmd.forward = axis_pwm

            if self.yaw_lock_enabled and self._telemetry_received:
                cmd.yaw = self.yaw_pid.control(self._yaw_error(), dt)
            self._update_centre_tracking(axis_err, now)
        else:
            if self.armed and not pose_fresh:
                self._warn_throttled(now, "[WARN] stale ArUco pose — holding neutral")
            self.lateral_pid.reset()
            self.forward_pid.reset()
            self.yaw_pid.reset()
            self._reset_centre_tracking()

        self.node.publish_commands(cmd)

        # Buffer plot samples — flushed to GUI at DISPLAY_HZ by _flush_plots.
        rel_t = now - self._t0
        plot_err = axis_err if axis_err is not None else 0.0
        if self.axis == "lateral":
            plot_pwm = cmd.lateral
        else:
            plot_pwm = cmd.forward
        with self._err_buf_lock:
            self._err_buf.append((rel_t, float(plot_err)))
            self._pwm_buf.append((rel_t, float(plot_pwm)))

    def _flush_plots(self):
        """Called on GUI thread at DISPLAY_HZ — drains buffered samples
        and emits them as a single batch signal."""
        with self._err_buf_lock:
            err_batch = self._err_buf
            pwm_batch = self._pwm_buf
            self._err_buf = []
            self._pwm_buf = []
        if err_batch:
            self.error_samples_batch.emit(err_batch)
        if pwm_batch:
            self.pwm_samples_batch.emit(pwm_batch)

    def _update_centre_tracking(self, err: float, now: float):
        in_band = abs(err) <= self.tolerance
        if not in_band:
            if self._in_band_since is not None:
                self.centred_status.emit(False, 0.0)
            self._in_band_since = None
            self._centred_already_fired = False
            return
        if self._in_band_since is None:
            self._in_band_since = now
        elapsed = now - self._in_band_since
        self.centred_status.emit(True, elapsed)
        if elapsed >= self.dwell_s and not self._centred_already_fired:
            self.log.emit(f"[CENTRE] reached in {elapsed:.2f}s")
            self.centred.emit(elapsed)
            self._centred_already_fired = True

    def _warn_throttled(self, now: float, msg: str):
        if (now - getattr(self, "_last_pose_warn", 0.0)) > 1.0:
            self.log.emit(msg)
            self._last_pose_warn = now


# ============================================================
# SECTION F — Qt GUI
# ============================================================

class GainRow(QWidget):
    """One PID gain: label + spinbox + slider with min/max labels."""

    value_changed = pyqtSignal(float)

    def __init__(self, name: str, vmin: float, vmax: float, init: float,
                 step: float):
        super().__init__()
        self._vmin = vmin
        self._vmax = vmax
        self._step = step
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        layout.addWidget(QLabel(f"<b>{name}</b>"))
        self.spin = QDoubleSpinBox()
        self.spin.setDecimals(3)
        self.spin.setRange(vmin, vmax)
        self.spin.setSingleStep(step)
        self.spin.setValue(init)
        self.spin.setFixedWidth(110)
        layout.addWidget(self.spin)

        layout.addWidget(QLabel(f"min {vmin:g}"))
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 1000)
        self.slider.setValue(self._value_to_slider(init))
        layout.addWidget(self.slider, 1)
        layout.addWidget(QLabel(f"max {vmax:g}"))

        self._guard = False
        self.spin.valueChanged.connect(self._on_spin)
        self.slider.valueChanged.connect(self._on_slider)

    def _value_to_slider(self, v: float) -> int:
        if self._vmax == self._vmin:
            return 0
        frac = (v - self._vmin) / (self._vmax - self._vmin)
        return int(round(max(0.0, min(1.0, frac)) * 1000))

    def _slider_to_value(self, s: int) -> float:
        return self._vmin + (s / 1000.0) * (self._vmax - self._vmin)

    def _on_spin(self, v: float):
        if self._guard:
            return
        self._guard = True
        self.slider.setValue(self._value_to_slider(v))
        self._guard = False
        self.value_changed.emit(v)

    def _on_slider(self, s: int):
        if self._guard:
            return
        v = self._slider_to_value(s)
        self._guard = True
        self.spin.setValue(v)
        self._guard = False
        self.value_changed.emit(v)

    def set_value(self, v: float):
        self._guard = True
        self.spin.setValue(v)
        self.slider.setValue(self._value_to_slider(v))
        self._guard = False


class TunerWindow(QMainWindow):
    def __init__(self, node: TunerNode, engine: ControlEngine,
                 grabber: VideoGrabber):
        super().__init__()
        self.setWindowTitle("Mira PID Tuner")
        self.node = node
        self.engine = engine
        self.grabber = grabber

        # GUI-side state
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_pose: Optional[MarkerPose] = None

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        top = QHBoxLayout()
        top.addWidget(self._build_video_panel(), 3)
        top.addWidget(self._build_control_panel(), 2)
        root.addLayout(top, 5)

        root.addWidget(self._build_gains_panel())
        root.addWidget(self._build_plots_panel(), 3)
        root.addWidget(self._build_log_panel(), 2)

        # Wire engine signals
        self.engine.log.connect(self._append_log)
        self.engine.error_samples_batch.connect(self._on_error_batch)
        self.engine.pwm_samples_batch.connect(self._on_pwm_batch)
        self.engine.centred_status.connect(self._on_centre_status)

        # Wire grabber
        self.grabber.frame_ready.connect(self._on_frame, Qt.QueuedConnection)
        self.grabber.status.connect(self._append_log, Qt.QueuedConnection)

        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self._refresh_plots)
        self._plot_timer.start(int(1000 / DISPLAY_HZ))

        # Apply initial config
        self._url_edit.setText(self.node.rtsp_url)
        self._id_spin.setValue(self.node.target_id)
        self.grabber.set_url(self.node.rtsp_url)
        self.engine.set_axis("lateral")

    # ---- panel builders ---------------------------------------------

    def _build_video_panel(self) -> QWidget:
        box = QGroupBox("Video")
        v = QVBoxLayout(box)
        self.video_label = QLabel("waiting for stream...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 360)
        self.video_label.setStyleSheet("background-color: #111; color: #888;")
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        v.addWidget(self.video_label)
        return box

    def _build_control_panel(self) -> QWidget:
        box = QGroupBox("Source / Mode")
        v = QVBoxLayout(box)

        v.addWidget(QLabel("RTSP / video URL:"))
        url_row = QHBoxLayout()
        self._url_edit = QLineEdit()
        url_row.addWidget(self._url_edit, 1)
        apply_url = QPushButton("Apply")
        apply_url.clicked.connect(self._on_apply_url)
        url_row.addWidget(apply_url)
        v.addLayout(url_row)

        id_row = QHBoxLayout()
        id_row.addWidget(QLabel("Target ArUco ID:"))
        self._id_spin = QSpinBox()
        self._id_spin.setRange(0, 1023)
        self._id_spin.valueChanged.connect(self._on_target_id)
        id_row.addWidget(self._id_spin)
        id_row.addStretch(1)
        v.addLayout(id_row)

        marker_row = QHBoxLayout()
        marker_row.addWidget(QLabel("Marker size (m):"))
        self._marker_spin = QDoubleSpinBox()
        self._marker_spin.setDecimals(3)
        self._marker_spin.setRange(0.01, 2.0)
        self._marker_spin.setSingleStep(0.01)
        self._marker_spin.setValue(MARKER_LENGTH)
        self._marker_spin.valueChanged.connect(self._on_marker_length)
        marker_row.addWidget(self._marker_spin)
        marker_row.addStretch(1)
        v.addLayout(marker_row)

        v.addWidget(self._hline())

        v.addWidget(QLabel("Axis:"))
        axis_row = QHBoxLayout()
        self._radio_lat = QRadioButton("Lateral")
        self._radio_fwd = QRadioButton("Forward")
        self._radio_lat.setChecked(True)
        grp = QButtonGroup(self)
        grp.addButton(self._radio_lat)
        grp.addButton(self._radio_fwd)
        self._radio_lat.toggled.connect(self._on_axis_change)
        axis_row.addWidget(self._radio_lat)
        axis_row.addWidget(self._radio_fwd)
        axis_row.addStretch(1)
        v.addLayout(axis_row)

        self._yaw_check = QCheckBox("Yaw heading lock")
        self._yaw_check.setChecked(True)
        self._yaw_check.toggled.connect(self.engine.set_yaw_lock)
        v.addWidget(self._yaw_check)

        v.addWidget(self._hline())

        self._arm_btn = QPushButton("ARM")
        self._arm_btn.setCheckable(True)
        self._arm_btn.setMinimumHeight(40)
        self._arm_btn.toggled.connect(self._on_arm_toggle)
        self._style_arm_button(False)
        v.addWidget(self._arm_btn)

        v.addWidget(self._hline())

        tol_row = QHBoxLayout()
        tol_row.addWidget(QLabel("Tolerance (m):"))
        self._tol_spin = QDoubleSpinBox()
        self._tol_spin.setDecimals(3)
        self._tol_spin.setRange(0.0, 5.0)
        self._tol_spin.setSingleStep(0.01)
        self._tol_spin.setValue(0.05)
        self._tol_spin.valueChanged.connect(self.engine.set_tolerance)
        tol_row.addWidget(self._tol_spin)
        v.addLayout(tol_row)

        dwell_row = QHBoxLayout()
        dwell_row.addWidget(QLabel("Dwell (s):"))
        self._dwell_spin = QDoubleSpinBox()
        self._dwell_spin.setDecimals(2)
        self._dwell_spin.setRange(0.0, 60.0)
        self._dwell_spin.setSingleStep(0.1)
        self._dwell_spin.setValue(1.5)
        self._dwell_spin.valueChanged.connect(self.engine.set_dwell)
        dwell_row.addWidget(self._dwell_spin)
        v.addLayout(dwell_row)

        self._centre_label = QLabel("Status: not centred")
        self._centre_label.setStyleSheet("padding: 4px;")
        v.addWidget(self._centre_label)

        v.addStretch(1)
        return box

    def _build_gains_panel(self) -> QWidget:
        box = QGroupBox("Gains (active axis)")
        v = QVBoxLayout(box)
        self._kp_row = GainRow("Kp", 0.0, 500.0, 200.0, 0.5)
        self._ki_row = GainRow("Ki", 0.0, 10.0, 0.0, 0.01)
        self._kd_row = GainRow("Kd", 0.0, 100.0, 0.0, 0.1)
        self._kp_row.value_changed.connect(lambda v: self._on_gain("kp", v))
        self._ki_row.value_changed.connect(lambda v: self._on_gain("ki", v))
        self._kd_row.value_changed.connect(lambda v: self._on_gain("kd", v))
        v.addWidget(self._kp_row)
        v.addWidget(self._ki_row)
        v.addWidget(self._kd_row)
        return box

    def _build_plots_panel(self) -> QWidget:
        box = QGroupBox("Lateral PWM — /master/commands (last 10 s)")
        v = QVBoxLayout(box)
        pg.setConfigOptions(antialias=True)

        self._pwm_plot = pg.PlotWidget()
        self._pwm_plot.showGrid(x=True, y=True, alpha=0.3)
        self._pwm_plot.setYRange(PWM_MIN - 50, PWM_MAX + 50)
        self._pwm_plot.setLabel("left", "PWM")
        self._pwm_plot.setLabel("bottom", "time (s, rolling)")
        self._pwm_curve = self._pwm_plot.plot(pen=pg.mkPen("c", width=2))
        self._pwm_plot.addLine(y=PWM_NEUTRAL, pen=pg.mkPen("w", style=Qt.DotLine))
        self._pwm_plot.addLine(y=PWM_MIN, pen=pg.mkPen("r", style=Qt.DashLine))
        self._pwm_plot.addLine(y=PWM_MAX, pen=pg.mkPen("r", style=Qt.DashLine))
        v.addWidget(self._pwm_plot)

        return box

    def _build_log_panel(self) -> QWidget:
        box = QGroupBox("Log")
        v = QVBoxLayout(box)
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(LOG_MAX_LINES)
        self._log.setStyleSheet("font-family: monospace;")
        v.addWidget(self._log)
        return box

    @staticmethod
    def _hline() -> QFrame:
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        return line

    def _style_arm_button(self, armed: bool):
        if armed:
            self._arm_btn.setText("DISARM")
            self._arm_btn.setStyleSheet(
                "background-color: #c62828; color: white; font-weight: bold;")
        else:
            self._arm_btn.setText("ARM")
            self._arm_btn.setStyleSheet(
                "background-color: #2e7d32; color: white; font-weight: bold;")

    # ---- slot handlers ----------------------------------------------

    def _on_apply_url(self):
        url = self._url_edit.text().strip()
        self.grabber.set_url(url)
        self._append_log(f"[VIDEO] URL set to {url}")

    def _on_target_id(self, value: int):
        self.engine.detector.set_target_id(value)
        self._append_log(f"[ARUCO] target id = {value}")

    def _on_axis_change(self, _checked: bool):
        axis = "lateral" if self._radio_lat.isChecked() else "forward"
        self.engine.set_axis(axis)
        self._reflect_axis_gains(axis)

    def _reflect_axis_gains(self, axis: str):
        pid = (self.engine.lateral_pid if axis == "lateral"
               else self.engine.forward_pid)
        self._kp_row.set_value(pid.kp)
        self._ki_row.set_value(pid.ki)
        self._kd_row.set_value(pid.kd)

    def _on_arm_toggle(self, checked: bool):
        self.engine.set_armed(checked)
        self._style_arm_button(checked)

    def _on_gain(self, name: str, value: float):
        axis = "lateral" if self._radio_lat.isChecked() else "forward"
        self.engine.set_gain(axis, name, value)

    def _on_marker_length(self, value: float):
        self.engine.detector.set_marker_length(value)
        self._append_log(f"[ARUCO] marker size = {value:.3f} m")

    def _append_log(self, msg: str):
        ts = _time.strftime("%H:%M:%S")
        line = f"{ts} {msg}"
        self._log.appendPlainText(line)
        # Also mirror to stdout so the launch terminal shows the same
        # messages as the in-window log panel.
        print(line, flush=True)

    @pyqtSlot(np.ndarray, object)
    def _on_frame(self, frame_bgr: np.ndarray, pose):
        # Detection already ran in the grabber thread; feed the pose to the
        # control engine and paint the frame immediately. Painting here
        # (instead of on a 15 Hz QTimer) shaves up to 67 ms of latency and
        # keeps the Qt event queue from accumulating frame_ready signals.
        self.engine.submit_pose(pose)
        self._latest_frame = frame_bgr
        self._latest_pose = pose
        self._refresh_video()

    def _refresh_video(self):
        if self._latest_frame is None:
            return
        frame = self._latest_frame
        h, w = frame.shape[:2]
        # downscale to fit the label while preserving aspect ratio
        target_w = max(320, self.video_label.width())
        target_h = max(180, self.video_label.height())
        scale = min(target_w / w, target_h / h)
        if scale < 1.0:
            new_w = max(1, int(w * scale))
            new_h = max(1, int(h * scale))
            frame = cv2.resize(frame, (new_w, new_h),
                               interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h2, w2 = rgb.shape[:2]
        img = QImage(rgb.data, w2, h2, w2 * 3, QImage.Format_RGB888).copy()
        self.video_label.setPixmap(QPixmap.fromImage(img))

    @pyqtSlot(list)
    def _on_error_batch(self, batch: list):
        for t, err in batch:
            self._error_t.append(t)
            self._error_y.append(err)

    @pyqtSlot(list)
    def _on_pwm_batch(self, batch: list):
        for t, pwm in batch:
            self._pwm_t.append(t)
            self._pwm_y.append(pwm)

    def _refresh_plots(self):
        # Read a snapshot of the ring buffer written by the ROS spin thread.
        pts = list(self.node.lateral_plot_buf)
        if not pts:
            return
        t_now = pts[-1][0]
        cutoff = t_now - PLOT_WINDOW_S
        pts = [(t, v) for t, v in pts if t >= cutoff]
        if not pts:
            return
        ts, vs = zip(*pts)
        # X-axis: seconds before now (negative = past), so the right edge is 0.
        ts_rel = [t - t_now for t in ts]
        self._pwm_curve.setData(list(ts_rel), list(vs))
        self._pwm_plot.setXRange(-PLOT_WINDOW_S, 0, padding=0)

    @pyqtSlot(bool, float)
    def _on_centre_status(self, in_band: bool, elapsed: float):
        if not in_band:
            self._centre_label.setText("Status: not centred")
            self._centre_label.setStyleSheet(
                "padding: 4px; background-color: #333; color: #ccc;")
            return
        dwell = self._dwell_spin.value()
        if elapsed < dwell:
            self._centre_label.setText(
                f"Status: in band {elapsed:.2f}/{dwell:.2f}s")
            self._centre_label.setStyleSheet(
                "padding: 4px; background-color: #ef6c00; color: white;")
        else:
            self._centre_label.setText(f"Status: centred for {elapsed:.2f}s")
            self._centre_label.setStyleSheet(
                "padding: 4px; background-color: #2e7d32; "
                "color: white; font-weight: bold;")

    def closeEvent(self, event):
        self.engine.stop()
        self.grabber.stop()
        self.grabber.wait(2000)
        super().closeEvent(event)


# ============================================================
# SECTION G — main()
# ============================================================

def _ros_spin_thread(node: Node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main():
    node = _NODE  # created at module level, before PyQt5
    calib_path = os.path.join(
        get_package_share_directory("mira2_perception"), "optimized_calib.npz")
    detector = ArucoDetector(calib_path, node.target_id)

    app = QApplication(sys.argv)

    grabber = VideoGrabber(detector)
    engine = ControlEngine(node, detector)
    win = TunerWindow(node, engine, grabber)
    win.resize(1400, 900)
    win.show()

    grabber.start()
    spin_thread = threading.Thread(target=_ros_spin_thread, args=(node,),
                                   daemon=True)
    spin_thread.start()

    try:
        rc = app.exec_()
    finally:
        engine.stop()
        grabber.stop()
        grabber.wait(2000)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
3D bounding box over detected valve — based on zed_yolo_xyz.py.

Identical pipeline to zed_yolo_xyz.py (same ZED thread, same point cloud
sampling, same AABB fit, same projection) with one change: the 2D green
rectangle is replaced by a 3D wireframe box drawn from the fitted AABB.

Usage:
    python zed_3d_bbox.py \
        --svo   /path/to/Main.svo2 \
        --model /path/to/best.pt   \
        [--conf 0.4] [--target valve] [--start 2800]
"""

import argparse
import math
import os
import threading
import time

import cv2
import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import ValveDetection

try:
    from ament_index_python.packages import get_package_share_directory
    _PKG_SHARE = get_package_share_directory("mira2_perception")
except Exception:
    _PKG_SHARE = None

_DEFAULT_MODEL = (
    os.path.join(_PKG_SHARE, "models", "valve_v1", "weights", "best.onnx")
    if _PKG_SHARE else None
)

# ── colours ───────────────────────────────────────────────────────────────────
CLR_BOX3D   = (0,   255, 255)   # yellow  — 3D wireframe
CLR_CTR     = (0,   255, 255)   # yellow  — centre dot
CLR_XYZ     = (0,   255, 255)   # yellow  — XYZ text
CLR_FPS     = (255, 255, 255)   # white   — HUD

GRID_STEPS  = 8
MIN_POINTS  = 6


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--svo",    required=True)
    p.add_argument("--model",  default=_DEFAULT_MODEL, required=_DEFAULT_MODEL is None)
    p.add_argument("--conf",   type=float, default=0.4)
    p.add_argument("--target", default="valve")
    p.add_argument("--start",  type=int,   default=2800,
                   help="SVO start frame (default 2800 = first valve frame)")
    return p.parse_args()


# ── ZED point cloud helpers ───────────────────────────────────────────────────

def get_xyz(pc: sl.Mat, cx: int, cy: int):
    err, val = pc.get_value(cx, cy)
    if err != sl.ERROR_CODE.SUCCESS:
        return None
    x, y, z = float(val[0]), float(val[1]), float(val[2])
    if any(math.isnan(v) or math.isinf(v) for v in (x, y, z)):
        return None
    return x, y, z


def sample_bbox_points(pc: sl.Mat, x1, y1, x2, y2, steps=GRID_STEPS):
    pts = []
    for px in np.linspace(x1, x2, steps, dtype=int):
        for py in np.linspace(y1, y2, steps, dtype=int):
            r = get_xyz(pc, int(px), int(py))
            if r:
                pts.append(r)
    return np.array(pts, dtype=np.float32) if pts else None


# ── 3D AABB ───────────────────────────────────────────────────────────────────

def fit_3d_bbox(pts):
    mn, mx   = pts.min(axis=0), pts.max(axis=0)
    centroid = (mn + mx) / 2.0
    half     = (mx - mn) / 2.0
    sx, sy, sz = half
    cx, cy, cz = centroid
    corners = np.array([
        [cx-sx, cy-sy, cz-sz], [cx+sx, cy-sy, cz-sz],
        [cx+sx, cy+sy, cz-sz], [cx-sx, cy+sy, cz-sz],
        [cx-sx, cy-sy, cz+sz], [cx+sx, cy-sy, cz+sz],
        [cx+sx, cy+sy, cz+sz], [cx-sx, cy+sy, cz+sz],
    ], dtype=np.float32)
    return centroid, half, corners


def project_corners(corners, cam_info):
    fx = cam_info.left_cam.fx
    fy = cam_info.left_cam.fy
    cx = cam_info.left_cam.cx
    cy = cam_info.left_cam.cy
    out = []
    for X, Y, Z in corners:
        if Z >= 0:          # Z is negative for objects in front (RIGHT_HANDED_Y_UP)
            out.append(None)
        else:
            depth = -Z      # positive depth
            out.append((int(fx * X / depth + cx),
                        int(-fy * Y / depth + cy)))
    return out


BOX_EDGES = [(0,1),(1,2),(2,3),(3,0),   # front face
             (4,5),(5,6),(6,7),(7,4),   # back face
             (0,4),(1,5),(2,6),(3,7)]   # pillars


def draw_3d_box(img, pts2d, colour=CLR_BOX3D, thickness=2):
    for i, j in BOX_EDGES:
        if pts2d[i] and pts2d[j]:
            cv2.line(img, pts2d[i], pts2d[j], colour, thickness, cv2.LINE_AA)


# ── ZED shared state + background thread ─────────────────────────────────────

class SharedState:
    def __init__(self):
        self.lock     = threading.Lock()
        self.bgr      = None
        self.pc       = None
        self.cam_info = None
        self.svo_pos  = 0
        self.total    = 0
        self.status   = "Initializing ZED..."
        self.ready    = False
        self.stop     = False
        self.skip_to  = None


def zed_worker(args, state: SharedState):
    zed  = sl.Camera()
    init = sl.InitParameters()
    init.set_from_svo_file(args.svo)
    init.svo_real_time_mode = False
    init.coordinate_units   = sl.UNIT.METER
    init.depth_mode         = sl.DEPTH_MODE.NEURAL
    init.coordinate_system  = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

    with state.lock:
        state.status = "Opening ZED SVO..."

    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        with state.lock:
            state.status = f"ZED OPEN FAILED: {err}"
        return

    cam_info = (zed.get_camera_information()
                   .camera_configuration
                   .calibration_parameters)
    with state.lock:
        state.cam_info = cam_info

    total = zed.get_svo_number_of_frames()
    zed.set_svo_position(args.start)

    with state.lock:
        state.total  = total
        state.status = "Optimizing NEURAL depth (one-time ~5 min)..."

    rt      = sl.RuntimeParameters()
    img_mat = sl.Mat()
    pc_mat  = sl.Mat()

    while not state.stop:
        with state.lock:
            sk = state.skip_to
            state.skip_to = None
        if sk is not None:
            zed.set_svo_position(sk)

        grab_err = zed.grab(rt)
        pos = zed.get_svo_position()

        if grab_err != sl.ERROR_CODE.SUCCESS:
            zed.set_svo_position(0)
            continue

        zed.retrieve_image(img_mat, sl.VIEW.LEFT)
        zed.retrieve_measure(pc_mat, sl.MEASURE.XYZRGBA)

        bgr     = cv2.cvtColor(img_mat.get_data(), cv2.COLOR_BGRA2BGR).copy()
        pc_copy = sl.Mat()
        pc_mat.copy_to(pc_copy)

        with state.lock:
            state.bgr     = bgr
            state.pc      = pc_copy
            state.svo_pos = pos
            state.status  = "running"
            state.ready   = True

    zed.close()


# ── main display loop ─────────────────────────────────────────────────────────

def main():
    args   = parse_args()
    target = args.target.lower()
    model  = YOLO(args.model)

    # ── ROS 2 setup ───────────────────────────────────────────────────────
    rclpy.init()
    ros_node = Node('valve_3d_bbox_node')
    det_pub  = ros_node.create_publisher(ValveDetection, '/valve/detection', 10)
    pose_pub = ros_node.create_publisher(PoseStamped,    '/valve/pose',      10)

    state = SharedState()
    t     = threading.Thread(target=zed_worker, args=(args, state), daemon=True)
    t.start()

    WIN = "ZED 3D BBox — valve"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 720)

    fps_t  = time.time()
    fps    = 0.0
    frames = 0
    paused = False

    print(f"[INFO] SVO    : {args.svo}")
    print(f"[INFO] Model  : {args.model}")
    print(f"[INFO] Target : '{target}'  conf≥{args.conf}")
    print(f"[INFO] Start  : frame {args.start}")
    print("[INFO] SPACE=pause  →=+200  ←=-200  Q/ESC=quit")

    while True:
        with state.lock:
            bgr      = state.bgr.copy() if state.bgr is not None else None
            pc       = state.pc
            svo_pos  = state.svo_pos
            total    = state.total
            status   = state.status
            ready    = state.ready

        # ── splash while ZED initialises ──────────────────────────────────
        if not ready or bgr is None:
            splash = np.zeros((720, 1280, 3), dtype=np.uint8)
            dots   = "." * (int(time.time() * 2) % 4)
            cv2.putText(splash, status + dots,
                        (60, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 220, 220), 2)
            cv2.putText(splash, "This happens only once per GPU — please wait",
                        (60, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
            cv2.imshow(WIN, splash)
            if cv2.waitKey(500) & 0xFF in (ord('q'), 27):
                break
            continue

        with state.lock:
            cam_info = state.cam_info
        img = bgr.copy() if not paused else img
        h, w = img.shape[:2]

        if not paused:
            results = model(img, conf=args.conf, verbose=False)[0]

            for box in results.boxes:
                cls_id   = int(box.cls[0])
                cls_name = model.names[cls_id].lower()
                conf_val = float(box.conf[0])
                if target != "all" and cls_name != target:
                    continue

                bx1, by1, bx2, by2 = map(int, box.xyxy[0])
                bx1 = max(0, min(bx1, w-1)); bx2 = max(0, min(bx2, w-1))
                by1 = max(0, min(by1, h-1)); by2 = max(0, min(by2, h-1))
                cx  = (bx1+bx2)//2;          cy  = (by1+by2)//2

                # class + confidence label
                label = f"{cls_name} {conf_val:.2f}"
                (lw, lh), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                cv2.rectangle(img, (bx1, by1-lh-8), (bx1+lw+4, by1),
                              CLR_BOX3D, -1)
                cv2.putText(img, label, (bx1+2, by1-4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)

                # centre dot
                cv2.circle(img, (cx, cy), 5, CLR_CTR, -1)

                # ── sample point cloud + fit 3D box ───────────────────────
                pts = sample_bbox_points(pc, bx1, by1, bx2, by2)
                if pts is not None and len(pts) >= MIN_POINTS:
                    centroid, half, corners = fit_3d_bbox(pts)

                    if cam_info:
                        pts2d = project_corners(corners, cam_info)
                        draw_3d_box(img, pts2d)

                    x3, y3, z3 = centroid
                    W, H, D    = half * 2
                    xyz_line  = f"X:{x3:+.2f} Y:{y3:+.2f} Z:{z3:.2f}m"
                    size_line = f"W:{W:.2f} H:{H:.2f} D:{D:.2f}m"
                    cv2.putText(img, xyz_line,  (bx1, by2+20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.52, CLR_XYZ, 2)
                    cv2.putText(img, size_line, (bx1, by2+40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.52, CLR_XYZ, 2)

                    # ── publish custom message ─────────────────────────────
                    stamp = ros_node.get_clock().now().to_msg()

                    det = ValveDetection()
                    det.header.stamp    = stamp
                    det.header.frame_id = 'zed_left_camera_frame'
                    det.class_name      = cls_name
                    det.confidence      = conf_val
                    det.x, det.y, det.z = float(x3), float(y3), float(z3)
                    det.length          = float(W)
                    det.breadth         = float(H)
                    det.height          = float(D)
                    det_pub.publish(det)

                    # /valve/pose for the controls node (camera frame)
                    pose = PoseStamped()
                    pose.header.stamp    = stamp
                    pose.header.frame_id = 'zed_left_camera_frame'
                    pose.pose.position.x = float(x3)
                    pose.pose.position.y = float(y3)
                    pose.pose.position.z = float(z3)
                    pose.pose.orientation.w = 1.0
                    pose_pub.publish(pose)
                else:
                    cv2.putText(img, "not enough depth pts",
                                (bx1, by2+20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 255), 1)

            frames += 1
            now = time.time()
            if now - fps_t >= 1.0:
                fps    = frames / (now - fps_t)
                frames = 0
                fps_t  = now

        cv2.putText(img, f"FPS: {fps:.1f}", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, CLR_FPS, 2)
        cv2.putText(img, f"Frame: {svo_pos}/{total}", (10, 54),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, CLR_FPS, 1)
        if paused:
            cv2.putText(img, "PAUSED", (w//2-60, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        cv2.imshow(WIN, img)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord(' '):
            paused = not paused
            print("[INFO] Paused" if paused else "[INFO] Resumed")
        if key == 83:
            new = min(svo_pos + 200, total-1)
            with state.lock:
                state.skip_to = new
            print(f"[INFO] Skip → frame {new}")
        if key == 81:
            new = max(svo_pos - 200, 0)
            with state.lock:
                state.skip_to = new
            print(f"[INFO] Skip ← frame {new}")

    with state.lock:
        state.stop = True
    t.join(timeout=3)
    cv2.destroyAllWindows()
    ros_node.destroy_node()
    rclpy.shutdown()
    print("[INFO] Done.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Underwater valve perception pipeline.

ZED runs on a background thread so cv2.imshow stays responsive even during
the one-time NEURAL depth model optimization (~5 min on first run).

Usage:
    python zed_yolo_xyz.py \
        --svo  /path/to/Main.svo2 \
        --model /path/to/best.pt  \
        [--conf 0.4] [--target valve] [--start 2800]
"""

import argparse
import math
import threading
import time

import cv2
import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO

# ── colours ──────────────────────────────────────────────────────────────────
CLR_BOX     = (0,   255,   0)
CLR_CTR     = (0,   255, 255)
CLR_XYZ_CTR = (0,   255, 255)
CLR_BOX3D   = (255, 128,   0)
CLR_XYZ_3D  = (255, 128,   0)
CLR_FPS     = (255, 255, 255)

GRID_STEPS  = 8
MIN_POINTS  = 6


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--svo",    required=True)
    p.add_argument("--model",  required=True)
    p.add_argument("--conf",   type=float, default=0.4)
    p.add_argument("--target", default="valve")
    p.add_argument("--start",  type=int,   default=2800,
                   help="SVO start frame (default 2800 = first valve frame)")
    p.add_argument("--no-3d",  action="store_true")
    return p.parse_args()


# ── ZED helpers ───────────────────────────────────────────────────────────────

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


# ── 3-D bbox ──────────────────────────────────────────────────────────────────

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
        if Z <= 0:
            out.append(None)
        else:
            out.append((int(fx*X/Z + cx), int(fy*Y/Z + cy)))
    return out


BOX_EDGES = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
             (0,4),(1,5),(2,6),(3,7)]


def draw_3d_bbox(img, pts2d, colour=CLR_BOX3D):
    for i, j in BOX_EDGES:
        if pts2d[i] and pts2d[j]:
            cv2.line(img, pts2d[i], pts2d[j], colour, 2)


# ── shared state between ZED thread and display thread ───────────────────────

class SharedState:
    def __init__(self):
        self.lock       = threading.Lock()
        self.bgr        = None          # latest left image
        self.pc         = None          # latest point cloud (sl.Mat)
        self.svo_pos    = 0
        self.total      = 0
        self.status     = "Initializing ZED..."
        self.ready      = False
        self.stop       = False
        self.skip_to    = None          # set to a frame number to seek


def zed_worker(args, state: SharedState):
    """Background thread: open ZED, grab frames, update shared state."""
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

    total = zed.get_svo_number_of_frames()
    zed.set_svo_position(args.start)

    with state.lock:
        state.total  = total
        state.status = "Optimizing NEURAL depth (one-time ~5 min)..."

    rt       = sl.RuntimeParameters()
    img_mat  = sl.Mat()
    pc_mat   = sl.Mat()

    while not state.stop:
        # handle seek requests from the display thread
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

        raw = img_mat.get_data()
        bgr = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR).copy()
        pc_copy = sl.Mat()
        pc_mat.copy_to(pc_copy)

        with state.lock:
            state.bgr     = bgr
            state.pc      = pc_copy
            state.svo_pos = pos
            state.status  = "running"
            state.ready   = True

    zed.close()


# ── main / display loop ───────────────────────────────────────────────────────

def main():
    args   = parse_args()
    target = args.target.lower()
    model  = YOLO(args.model)

    cam_info_holder = [None]   # filled after first ZED open

    state  = SharedState()
    t      = threading.Thread(target=zed_worker, args=(args, state), daemon=True)
    t.start()

    WIN = "ZED YOLO XYZ — valve perception"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 720)

    fps_t  = time.time()
    fps    = 0.0
    frames = 0
    paused = False

    print(f"[INFO] SVO      : {args.svo}")
    print(f"[INFO] Model    : {args.model}")
    print(f"[INFO] Target   : '{target}'  conf≥{args.conf}")
    print(f"[INFO] Start    : frame {args.start}")
    print("[INFO] Controls : SPACE=pause  →=+200  ←=-200  Q/ESC=quit")

    while True:
        with state.lock:
            bgr     = state.bgr.copy()   if state.bgr is not None else None
            pc      = state.pc
            svo_pos = state.svo_pos
            total   = state.total
            status  = state.status
            ready   = state.ready

        # ── waiting / optimization screen ────────────────────────────────
        if not ready or bgr is None:
            placeholder = np.zeros((720, 1280, 3), dtype=np.uint8)
            dots = "." * (int(time.time() * 2) % 4)
            cv2.putText(placeholder, status + dots,
                        (60, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 220, 220), 2)
            cv2.putText(placeholder, "This happens only once per GPU — please wait",
                        (60, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
            cv2.imshow(WIN, placeholder)
            key = cv2.waitKey(500) & 0xFF
            if key in (ord('q'), 27):
                break
            continue

        # ── grab camera intrinsics once ───────────────────────────────────
        if cam_info_holder[0] is None:
            # Re-open a temporary camera just to read intrinsics from the SVO
            tmp = sl.Camera()
            ip  = sl.InitParameters()
            ip.set_from_svo_file(args.svo)
            ip.depth_mode = sl.DEPTH_MODE.NONE
            if tmp.open(ip) == sl.ERROR_CODE.SUCCESS:
                cam_info_holder[0] = (
                    tmp.get_camera_information()
                       .camera_configuration
                       .calibration_parameters
                )
                tmp.close()

        cam_info = cam_info_holder[0]
        img = bgr.copy() if not paused else img
        h, w = img.shape[:2]

        if not paused:
            # ── YOLO ─────────────────────────────────────────────────────
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

                cv2.rectangle(img, (bx1, by1), (bx2, by2), CLR_BOX, 2)
                cv2.circle(img, (cx, cy), 5, CLR_CTR, -1)

                label = f"{cls_name} {conf_val:.2f}"
                (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                cv2.rectangle(img, (bx1, by1-lh-8), (bx1+lw+4, by1), CLR_BOX, -1)
                cv2.putText(img, label, (bx1+2, by1-4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,0), 1)

                # Method 1 — centre depth
                xyz_ctr = get_xyz(pc, cx, cy) if pc else None
                if xyz_ctr:
                    x, y, z = xyz_ctr
                    t1 = f"CTR  X:{x:+.2f} Y:{y:+.2f} Z:{z:.2f}m"
                else:
                    t1 = "CTR  depth: invalid"
                cv2.putText(img, t1, (bx1, by2+18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, CLR_XYZ_CTR, 2)

                # Method 2 — 3D cuboid
                if not args.no_3d and pc:
                    pts = sample_bbox_points(pc, bx1, by1, bx2, by2)
                    if pts is not None and len(pts) >= MIN_POINTS:
                        centroid, half, corners = fit_3d_bbox(pts)
                        if cam_info:
                            pts2d = project_corners(corners, cam_info)
                            draw_3d_bbox(img, pts2d)
                        x3, y3, z3 = centroid
                        sx, sy, sz  = half*2
                        t2 = (f"3D   X:{x3:+.2f} Y:{y3:+.2f} Z:{z3:.2f}m "
                              f"[{sx:.2f}x{sy:.2f}x{sz:.2f}]")
                    else:
                        t2 = "3D   not enough valid pts"
                    cv2.putText(img, t2, (bx1, by2+38),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.52, CLR_XYZ_3D, 2)

            # FPS
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
        cv2.putText(img, "GREEN=CentreDepth  ORANGE=3D-Cuboid", (10, h-12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1)
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
        if key == 83:   # →
            new = min(svo_pos + 200, total-1)
            with state.lock:
                state.skip_to = new
            print(f"[INFO] Skip → frame {new}")
        if key == 81:   # ←
            new = max(svo_pos - 200, 0)
            with state.lock:
                state.skip_to = new
            print(f"[INFO] Skip ← frame {new}")

    with state.lock:
        state.stop = True
    t.join(timeout=3)
    cv2.destroyAllWindows()
    print("[INFO] Done.")


if __name__ == "__main__":
    main()

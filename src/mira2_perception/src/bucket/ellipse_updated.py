import cv2
import numpy as np
from collections import deque

cap = cv2.VideoCapture("vid1.mp4")

LOWER_BLUE = np.array([105, 40, 50])
UPPER_BLUE = np.array([125, 255, 255])

MIN_BUCKET_AREA = 3000
SMOOTH_N = 5
cx_history = deque(maxlen=SMOOTH_N)
cy_history = deque(maxlen=SMOOTH_N)

ellipse = None


def is_valid_ellipse(ellipse, frame_shape):
    (cx, cy), (ma, Mi), angle = ellipse
    h, w = frame_shape[:2]

    if not (0 < cx < w and 0 < cy < h):
        return False
    if ma < 10 or Mi < 10:
        return False

    ratio = max(ma, Mi) / (min(ma, Mi) + 1e-5)
    if ratio > 4.0:
        return False

    return True


def detect_buckets(frame):
    global ellipse

    h, w = frame.shape[:2]
    frame_cx, frame_cy = w // 2, h // 2

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blue_mask = cv2.inRange(frame_hsv, LOWER_BLUE, UPPER_BLUE)

    kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel_small)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel_large)

    contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    vis = frame.copy()
    detected = False

    if contours:
        valid_contours = [c for c in contours if cv2.contourArea(c) > MIN_BUCKET_AREA]

        if valid_contours:
            largest = max(valid_contours, key=cv2.contourArea)

            if len(largest) >= 5:
                candidate = cv2.fitEllipse(largest)

                if is_valid_ellipse(candidate, frame.shape):
                    ellipse = candidate
                    (cx, cy), axes, angle = ellipse
                    cx_history.append(int(cx))
                    cy_history.append(int(cy))
                    detected = True

    if cx_history:
        smooth_cx = int(np.mean(cx_history))
        smooth_cy = int(np.mean(cy_history))

        offset_x = smooth_cx - frame_cx
        offset_y = smooth_cy - frame_cy
        norm_x = offset_x / (w / 2)
        norm_y = offset_y / (h / 2)

        # Draw ellipse
        if ellipse is not None: 
            color = (255, 0, 0) if detected else (0, 0, 255)
            cv2.ellipse(vis, ellipse, color, 2)

        # Draw smoothed center
        cv2.circle(vis, (smooth_cx, smooth_cy), 8, (0, 255, 0), -1)

        # Draw frame center
        cv2.drawMarker(vis, (frame_cx, frame_cy),
                       (255, 255, 255), cv2.MARKER_CROSS, 20, 2)

        # Draw offset line
        cv2.line(vis, (frame_cx, frame_cy),
                 (smooth_cx, smooth_cy), (0, 255, 255), 2)

        # HUD
        status = "DETECTED" if detected else "LOST (last known)"
        status_color = (0, 255, 0) if detected else (0, 165, 255)

        cv2.putText(vis, status,
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(vis, f"Offset px  : ({offset_x:+d}, {offset_y:+d})",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis, f"Offset norm: ({norm_x:+.2f}, {norm_y:+.2f})",
                    (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return offset_x, offset_y, norm_x, norm_y, vis, blue_mask

    # Nothing detected yet, still draw frame center
    cv2.drawMarker(vis, (frame_cx, frame_cy),
                   (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
    cv2.putText(vis, "NO DETECTION",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    return None, None, None, None, vis, blue_mask


while True:
    ret, frame = cap.read()
    if not ret:
        break

    offset_x, offset_y, norm_x, norm_y, vis, blue_mask = detect_buckets(frame)

    
    cv2.imshow("Blue Mask", blue_mask)
    cv2.imshow("Bucket Detection", vis)

    key = cv2.waitKey(0) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()




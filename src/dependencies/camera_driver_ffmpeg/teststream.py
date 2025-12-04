import cv2
import os

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "fifo_size;500000|overrun_nonfatal;1|fflags;nobuffer|flags;low_delay|framedrop;1"
)


# UDP MJPEG stream URL with FFmpeg low-latency options
url = input("Enter the UDP MJPEG stream URL (e.g., udp://127.0.0.1:8554): ")

cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)

if not cap.isOpened():
    raise RuntimeError("Cannot open video stream")

while True:
    ret, frame = cap.read()
    if not ret:
        continue  # drop if frame not ready

    cv2.imshow("MJPEG UDP (FFmpeg)", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

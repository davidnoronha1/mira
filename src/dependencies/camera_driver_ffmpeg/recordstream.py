import cv2
import os
from datetime import datetime


os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "fifo_size;500000|overrun_nonfatal;1|fflags;nobuffer|flags;low_delay|framedrop;1"
)


# UDP MJPEG stream URL with FFmpeg low-latency options
url = input("Enter the UDP MJPEG stream URL (e.g., udp://127.0.0.1:8554): ")


cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)


if not cap.isOpened():
    raise RuntimeError("Cannot open video stream")


# Get stream properties
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)
if fps == 0 or fps > 120:  # fallback if FPS not detected
    fps = 30.0


# Generate filename with timestamp
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_filename = f"recorded_stream_{timestamp}.avi"


# Define codec and create VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))


if not out.isOpened():
    raise RuntimeError("Cannot open VideoWriter")


print(f"Recording to: {output_filename}")
print(f"Resolution: {frame_width}x{frame_height}, FPS: {fps}")
print("Press 'q' to stop recording")


frame_count = 0


while True:
    ret, frame = cap.read()
    if not ret:
        continue  # drop if frame not ready


    # Write frame to file
    out.write(frame)
    frame_count += 1


    # Display frame with recording indicator
    cv2.putText(frame, f"REC | Frames: {frame_count}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.imshow("MJPEG UDP Recorder (FFmpeg)", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# Release resources
cap.release()
out.release()
cv2.destroyAllWindows()


print(f"\nRecording saved: {output_filename}")
print(f"Total frames recorded: {frame_count}")

#!/usr/bin/env python3
"""
Loop a video over RTSP using OpenCV for reading + GStreamer RTSP Server for serving.

OpenCV reads and loops the video file frame by frame.
Frames are pushed into a GStreamer appsrc element.
GStreamer encodes (x264) and serves over RTSP.

Requirements:
    pip install opencv-python
    sudo apt install python3-gi gir1.2-gst-rtsp-server-1.0 \
        gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav gstreamer1.0-rtsp

Usage:
    python publish_rtsp_stream.py /path/to/video.mp4
    python publish_rtsp_stream.py /path/to/video.mp4 --port 2001 --endpoint image_rtsp
"""

import argparse
import os
import sys
import signal
import threading
import time

import cv2

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
gi.require_version("GLib", "2.0")
from gi.repository import Gst, GstRtspServer, GLib


# ─── Args ─────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Stream a looping video over RTSP (OpenCV reader + GStreamer RTSP server)."
    )
    parser.add_argument("video",                               help="Path to video file")
    parser.add_argument("--port",     type=int, default=2001, help="RTSP port (default: 2001)")
    parser.add_argument("--endpoint", default="image_rtsp",   help="RTSP endpoint (default: image_rtsp)")
    parser.add_argument("--bitrate",  type=int, default=8000, help="x264 bitrate kbps (default: 8000)")
    parser.add_argument("--latency",  type=int, default=200,  help="RTSP jitter buffer ms (default: 200)")
    parser.add_argument("--width",    type=int, default=None, help="Resize width (default: source)")
    parser.add_argument("--height",   type=int, default=None, help="Resize height (default: source)")
    return parser.parse_args()


# ─── OpenCV frame producer ────────────────────────────────────────────────────

class VideoLooper:
    """
    Reads a video file with OpenCV in a background thread, looping forever.
    Frames are pushed into a GStreamer appsrc element as GstBuffers.
    """

    def __init__(self, video_path: str, appsrc: Gst.Element,
                 width: int, height: int, fps: float):
        self.video_path   = video_path
        self.appsrc       = appsrc
        self.width        = width
        self.height       = height
        self.fps          = fps
        self.frame_dur_ns = int(1e9 / fps)

        self._running = False
        self._paused  = False          # controlled by need-data / enough-data
        self._thread  = None
        self._pts     = 0              # monotonic presentation timestamp (ns)

        self.appsrc.connect("need-data",   self._cb_need_data)
        self.appsrc.connect("enough-data", self._cb_enough_data)

    def _cb_need_data(self, src, length):
        self._paused = False

    def _cb_enough_data(self, src):
        self._paused = True

    def _push_frame(self, frame):
        """Encode one BGR frame and push it to appsrc. Returns False on error."""
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height),
                               interpolation=cv2.INTER_LINEAR)
        yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV_I420)
        buf = Gst.Buffer.new_wrapped(yuv.tobytes())
        buf.pts      = self._pts
        buf.duration = self.frame_dur_ns
        self._pts   += self.frame_dur_ns
        flow = self.appsrc.emit("push-buffer", buf)
        if flow != Gst.FlowReturn.OK:
            print(f"[WARN] push-buffer: {flow}")
            self._running = False
            return False
        return True

    def _push_loop(self):
        frame_interval = 1.0 / self.fps
        is_image = self.video_path.lower().endswith(".png")

        if is_image:
            frame = cv2.imread(self.video_path)
            if frame is None:
                print(f"[ERROR] Cannot read image: {self.video_path}", file=sys.stderr)
                return
            print("[INFO] Image mode — looping single frame.")
            while self._running:
                t0 = time.monotonic()
                while self._running and self._paused:
                    time.sleep(0.005)
                if not self._running:
                    break
                if not self._push_frame(frame):
                    break
                elapsed = time.monotonic() - t0
                sleep = frame_interval - elapsed
                if sleep > 0:
                    time.sleep(sleep)
            return

        while self._running:
            cap = cv2.VideoCapture(self.video_path)
            if not cap.isOpened():
                print(f"[ERROR] Cannot open video: {self.video_path}", file=sys.stderr)
                return

            while self._running:
                t0 = time.monotonic()

                ret, frame = cap.read()
                if not ret:
                    # EOF — break inner loop to reopen (loop)
                    print("[INFO] EOF — looping.")
                    break

                # Back-pressure: wait if appsrc has enough data
                while self._running and self._paused:
                    time.sleep(0.005)

                if not self._running:
                    break

                if not self._push_frame(frame):
                    break

                # Sleep for remainder of frame interval
                elapsed = time.monotonic() - t0
                sleep   = frame_interval - elapsed
                if sleep > 0:
                    time.sleep(sleep)

            cap.release()

    def start(self):
        self._running = True
        self._paused  = True   # wait for need-data signal before pushing
        self._thread  = threading.Thread(target=self._push_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=5)


# ─── Custom RTSP media factory ────────────────────────────────────────────────

class AppsrcMediaFactory(GstRtspServer.RTSPMediaFactory):
    """
    Builds the GStreamer pipeline with appsrc as video source.
    set_shared(True): one pipeline, one OpenCV reader, all clients share it.
    The pipeline is created once on first client connect.
    """

    def __init__(self, video_path: str,
                 width: int, height: int, fps: float,
                 bitrate: int, latency: int):
        super().__init__()
        self.video_path = video_path
        self.width      = width
        self.height     = height
        self.fps        = fps
        self.bitrate    = bitrate
        self._looper    = None

        self.set_shared(True)
        self.set_latency(latency)

    def do_create_element(self, url):
        """Called once (shared=True) to build the media pipeline."""

        fps_n = int(self.fps)           # numerator   (e.g. 30)
        fps_d = 1                       # denominator

        pipeline_str = (
            f"appsrc name=vsrc is-live=true format=time do-timestamp=false "
            f"caps=\"video/x-raw,format=I420,"
            f"width={self.width},height={self.height},"
            f"framerate={fps_n}/{fps_d}\" ! "
            "queue max-size-buffers=10 leaky=downstream ! "
            f"x264enc tune=zerolatency speed-preset=superfast "
            # key-int-max set to ~half second so the decoder recovers
            # quickly when a P-frame gets corrupted in transit.
            f"bitrate={self.bitrate} key-int-max=10 bframes=0 " 
            f"byte-stream=true aud=true ! "
            "video/x-h264,profile=baseline ! "
            "h264parse ! "
            "rtph264pay name=pay0 pt=96 config-interval=1"
        )

        print(f"[INFO] Pipeline:\n  {pipeline_str}\n")

        pipeline = Gst.parse_bin_from_description(pipeline_str, False)
        appsrc   = pipeline.get_by_name("vsrc")

        # Ensure appsrc properties are set even if parse_bin_from_description
        # doesn't honour them from the string (known Python binding quirk)
        appsrc.set_property("is-live", True)
        appsrc.set_property("format", Gst.Format.TIME)
        appsrc.set_property("do-timestamp", False)

        # Stop any previous looper (shouldn't happen with shared=True, but be safe)
        if self._looper is not None:
            self._looper.stop()

        self._looper = VideoLooper(
            video_path=self.video_path,
            appsrc=appsrc,
            width=self.width,
            height=self.height,
            fps=self.fps,
        )
        self._looper.start()
        print("[INFO] VideoLooper started.")

        return pipeline

    def cleanup(self):
        if self._looper:
            self._looper.stop()
            self._looper = None


# ─── Top-level server ─────────────────────────────────────────────────────────

class RTSPLoopServer:
    def __init__(self, args):
        self.video_path = os.path.abspath(args.video)
        self.port       = args.port
        self.endpoint   = args.endpoint.lstrip("/")
        self.bitrate    = args.bitrate
        self.latency    = args.latency

        Gst.init(None)

        # Probe source dimensions + FPS with OpenCV
        if self.video_path.lower().endswith(".png"):
            img = cv2.imread(self.video_path)
            if img is None:
                print(f"[ERROR] Cannot read image: {self.video_path}", file=sys.stderr)
                sys.exit(1)
            src_h, src_w = img.shape[:2]
            src_fps = 30.0
        else:
            cap = cv2.VideoCapture(self.video_path)
            if not cap.isOpened():
                print(f"[ERROR] Cannot open: {self.video_path}", file=sys.stderr)
                sys.exit(1)
            src_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            src_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
            cap.release()

        # Pick output dimensions:
        #   - both --width and --height given → use as-is
        #   - only one given → preserve aspect ratio
        #   - neither given → auto-downscale to 960 wide if source is bigger
        #     (bigger sources at the default 4 Mbps overrun x264 in
        #     zerolatency/veryfast mode and produce decoder corruption)
        AUTO_MAX_WIDTH = 960
        if args.width is not None and args.height is not None:
            w, h = args.width, args.height
        elif args.width is not None:
            w = args.width
            h = int(round(src_h * (w / src_w)))
        elif args.height is not None:
            h = args.height
            w = int(round(src_w * (h / src_h)))
        elif src_w > AUTO_MAX_WIDTH:
            w = AUTO_MAX_WIDTH
            h = int(round(src_h * (w / src_w)))
        else:
            w, h = src_w, src_h
        # x264 needs even dims
        self.width  = w + (w % 2)
        self.height = h + (h % 2)
        self.fps    = min(src_fps, 20.0)

        print(f"[INFO] Source  : {src_w}x{src_h} @ {src_fps:.3f} fps")
        print(f"[INFO] Output  : {self.width}x{self.height} @ {self.fps:.3f} fps")

        self._factory  = None
        self._glib_loop = GLib.MainLoop()

    def run(self):
        server = GstRtspServer.RTSPServer()
        server.set_service(str(self.port))

        self._factory = AppsrcMediaFactory(
            video_path=self.video_path,
            width=self.width,
            height=self.height,
            fps=self.fps,
            bitrate=self.bitrate,
            latency=self.latency,
        )

        mounts = server.get_mount_points()
        mounts.add_factory(f"/{self.endpoint}", self._factory)
        server.attach(None)

        url = f"rtsp://localhost:{self.port}/{self.endpoint}"
        print(f"[INFO] RTSP URL : {url}")
        print(f"[INFO] Bitrate  : {self.bitrate} kbps")
        print(f"[INFO] Latency  : {self.latency} ms")
        print(f"\n       ffplay {url}")
        print(f"       vlc    {url}")
        print(f"\n[INFO] Ctrl+C to stop.\n")

        def _shutdown(sig, frame):
            print("\n[INFO] Shutting down...")
            if self._factory:
                self._factory.cleanup()
            self._glib_loop.quit()

        signal.signal(signal.SIGINT, _shutdown)
        signal.signal(signal.SIGTERM, _shutdown)
        self._glib_loop.run()
        print("[INFO] Done.")


# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    args = parse_args()
    if not os.path.isfile(args.video):
        print(f"[ERROR] File not found: {args.video}", file=sys.stderr)
        sys.exit(1)
    RTSPLoopServer(args).run()


if __name__ == "__main__":
    main()

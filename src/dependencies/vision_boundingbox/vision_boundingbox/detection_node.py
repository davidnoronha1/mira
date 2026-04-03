#!/usr/bin/env python3
"""ROS2 node for YOLO object detection."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import threading
from queue import Queue
import time

from .yolo_detector import YOLODetector


class VisionBoundingBoxNode(Node):
    """ROS2 node for YOLO-based object detection."""
    
    def __init__(self):
        super().__init__('vision_boundingbox_node')
        
        self.get_logger().info('Vision Bounding Box Node starting up.')
        self.get_logger().info('This node detects objects from a camera or image topic.')
        
        # Declare parameters
        self.declare_parameter('webcam', False)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('rtsp_url', '')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('device', 'CPU')
        self.declare_parameter('input_height', 640)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('visualize', False)
        self.declare_parameter('publish_image', False)
        self.declare_parameter('model_name', 'yolo11n.onnx')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.4)
        self.declare_parameter('reject_reflections', True)
        self.declare_parameter('reject_threshold', 0.5)
        
        # Get parameters
        self.webcam = self.get_parameter('webcam').value
        self.camera_index = self.get_parameter('camera_index').value
        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.input_topic = self.get_parameter('input_topic').value
        self.device = self.get_parameter('device').value
        self.input_height = self.get_parameter('input_height').value
        self.input_width = self.get_parameter('input_width').value
        self.visualize = self.get_parameter('visualize').value
        self.publish_image = self.get_parameter('publish_image').value
        self.model_name = self.get_parameter('model_name').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.reject_reflections = self.get_parameter('reject_reflections').value
        self.reject_threshold = self.get_parameter('reject_threshold').value
        
        self.get_logger().info(f'Device: {self.device}')
        self.get_logger().info(f'Input size: {self.input_width}x{self.input_height}')
        self.get_logger().info(f'Model: {self.model_name}')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load model
        package_share_dir = get_package_share_directory('vision_boundingbox')
        model_path = Path(package_share_dir) / 'models' / self.model_name
        class_names_path = Path(package_share_dir) / 'models' / 'class_names.txt'
        
        try:
            self.detector = YOLODetector(
                str(model_path),
                str(class_names_path) if class_names_path.exists() else None,
                input_size=(self.input_height, self.input_width),
                conf_threshold=self.conf_threshold,
                nms_threshold=self.nms_threshold,
                device=self.device
            )
            self.get_logger().info('YOLO detector initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize YOLO detector: {e}')
            raise
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/vision/detections',
            10
        )
        
        if self.publish_image:
            self.image_pub = self.create_publisher(
                Image,
                '/vision/detections/image',
                10
            )
        
        # Service for start/stop
        self.start_stop_srv = self.create_service(
            SetBool,
            '~/start_stop',
            self.start_stop_callback
        )
        
        # Processing state
        self.processing = False
        self.cap = None
        self.timer = None
        self.image_sub = None
        self.using_video_capture = False  # True for webcam or RTSP
        
        # Processing queue
        self.frame_queue = Queue(maxsize=3)
        self.processing_thread = None
        self.shutdown_event = threading.Event()
        
        # Start processing
        self.start()
    
    def start_stop_callback(self, request, response):
        """Service callback for start/stop."""
        if request.data:
            self.start()
            response.message = "Detection started."
        else:
            self.stop()
            response.message = "Detection stopped."
        response.success = True
        return response
    
    def start(self):
        """Start detection."""
        if self.processing:
            self.get_logger().info('Detection is already running.')
            return
        
        self.get_logger().info('Starting detection...')
        self.processing = True
        
        # Start processing thread
        self.shutdown_event.clear()
        self.processing_thread = threading.Thread(target=self.process_loop, daemon=True)
        self.processing_thread.start()
        
        # Check for RTSP stream first, then webcam, then topic
        if self.rtsp_url:
            # Open RTSP stream
            self.get_logger().info(f'Opening RTSP stream: {self.rtsp_url}')
            self.cap = cv2.VideoCapture(self.rtsp_url)
            if not self.cap.isOpened():
                self.get_logger().error(f'Failed to open RTSP stream: {self.rtsp_url}')
                self.processing = False
                return
            
            self.using_video_capture = True
            # Create timer for reading frames
            self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS
            self.get_logger().info('RTSP stream opened successfully')
            
        elif self.webcam:
            # Open webcam
            self.get_logger().info(f'Opening webcam at index {self.camera_index}')
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                self.get_logger().error(f'Failed to open camera {self.camera_index}')
                self.processing = False
                return
            
            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            self.using_video_capture = True
            # Create timer for webcam reading
            self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS
            self.get_logger().info('Webcam opened successfully')
            
        else:
            # Subscribe to image topic
            self.get_logger().info(f'Subscribing to image topic: {self.input_topic}')
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            self.image_sub = self.create_subscription(
                Image,
                self.input_topic,
                self.image_callback,
                qos
            )
            self.using_video_capture = False
        
        self.get_logger().info('Detection started successfully.')
    
    def stop(self):
        """Stop detection."""
        if not self.processing:
            self.get_logger().info('Detection is already stopped.')
            return
        
        self.get_logger().info('Stopping detection...')
        self.processing = False
        
        # Stop input sources
        if self.using_video_capture and self.cap is not None:
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            self.cap.release()
            self.cap = None
            self.using_video_capture = False
        elif self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        
        # Stop processing thread
        self.shutdown_event.set()
        if self.processing_thread is not None:
            self.processing_thread.join(timeout=2.0)
            self.processing_thread = None
        
        # Clear queue
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except:
                break
        
        if self.visualize:
            cv2.destroyAllWindows()
        
        self.get_logger().info('Detection stopped successfully.')
    
    def timer_callback(self):
        """Timer callback for webcam/RTSP stream."""
        if not self.processing or self.cap is None:
            return
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to read frame from video source')
            return
        
        self.process_frame(frame)
    
    def image_callback(self, msg):
        """Image topic callback."""
        if not self.processing:
            return
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
    
    def process_frame(self, frame):
        """Add frame to processing queue."""
        if not self.frame_queue.full():
            try:
                self.frame_queue.put_nowait(frame.copy())
            except:
                pass
    
    def _simple_reject_reflections(self, detections, threshold=0.5):
        classes = dict()
        for det in detections:
            if det.class_id not in classes:
                classes[det.class_id] = det
            else:
                x1, y1, w1, h1 = classes[det.class_id].box
                x2, y2, w2, h2 = det.box
                score = 0.8 * (y2-y1)/self.input_height + 0.2 * ((self.input_height/2)-y2)/self.input_height
                #score = (det.confidence/classes[det.class_id].confidence) * score
                if score > threshold:
                    classes[det.class_id] = det
        return list(classes.values())

    def process_loop(self):
        """Processing loop running in separate thread."""
        while not self.shutdown_event.is_set():
            try:
                # Get frame from queue with timeout
                frame = self.frame_queue.get(timeout=0.1)
            except:
                continue
            
            try:
                # Run detection
                detections = self.detector.detect(frame)
                if self.reject_reflections:
                    detections = self._simple_reject_reflections(detections)
                
                # Publish detections
                detection_msg = Detection2DArray()
                detection_msg.header.stamp = self.get_clock().now().to_msg()
                detection_msg.header.frame_id = 'camera'
                
                for det in detections:
                    detection = Detection2D()
                    detection.id = det.class_id
                    detection.results.append(det.confidence)
                    x, y, w, h = det.box
                    detection.bbox.center.position.x = float(x + w / 2.0)
                    detection.bbox.center.position.y = float(y + h / 2.0)
                    detection.bbox.size_x = float(w)
                    detection.bbox.size_y = float(h)
                    detection_msg.detections.append(detection)
                
                self.detection_pub.publish(detection_msg)
                
                # Draw and publish/visualize if needed
                if self.publish_image or self.visualize:
                    result_img = self.detector.draw_detections(frame, detections)
                    
                    if self.publish_image:
                        result_msg = self.bridge.cv2_to_imgmsg(result_img, 'bgr8')
                        result_msg.header = detection_msg.header
                        self.image_pub.publish(result_msg)
                    
                    if self.visualize:
                        cv2.imshow('YOLO Detections', result_img)
                        cv2.waitKey(1)
                
            except Exception as e:
                self.get_logger().error(f'Error processing frame: {e}')
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        self.stop()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = VisionBoundingBoxNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

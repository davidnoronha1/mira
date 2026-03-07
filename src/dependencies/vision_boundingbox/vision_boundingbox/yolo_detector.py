#!/usr/bin/env python3
"""YOLO object detector using ONNX Runtime."""

import numpy as np
import cv2
import onnxruntime as ort
from pathlib import Path
from typing import List, Tuple, Optional
import time


class Detection:
    """Detection result."""
    def __init__(self, box, confidence, class_id, class_name):
        self.box = box  # (x, y, w, h)
        self.confidence = confidence
        self.class_id = class_id
        self.class_name = class_name


class YOLODetector:
    """YOLO object detector using ONNX Runtime."""
    
    def __init__(self, model_path: str, class_names_path: Optional[str] = None, 
                 input_size: Tuple[int, int] = (640, 640),
                 conf_threshold: float = 0.5, nms_threshold: float = 0.4,
                 device: str = 'CPU'):
        """
        Initialize YOLO detector.
        
        Args:
            model_path: Path to ONNX model file
            class_names_path: Path to class names file (one class per line)
            input_size: Model input size (height, width)
            conf_threshold: Confidence threshold for detections
            nms_threshold: NMS IoU threshold
            device: Device to use ('CPU' or 'CUDA')
        """
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        
        # Load class names
        self.class_names = self._load_class_names(class_names_path)
        
        # Create ONNX Runtime session
        providers = []
        if device.upper() in ['CUDA', 'GPU']:
            providers.append('CUDAExecutionProvider')
        providers.append('CPUExecutionProvider')
        
        self.session = ort.InferenceSession(model_path, providers=providers)
        
        # Get model input/output names
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        
        print(f"Model loaded: {model_path}")
        print(f"Input shape: {self.session.get_inputs()[0].shape}")
        print(f"Output shape: {self.session.get_outputs()[0].shape}")
        print(f"Execution providers: {self.session.get_providers()}")
    
    def _load_class_names(self, class_names_path: Optional[str]) -> List[str]:
        """Load class names from file."""
        if class_names_path and Path(class_names_path).exists():
            with open(class_names_path, 'r') as f:
                return [line.strip() for line in f if line.strip()]
        
        # Default COCO classes
        return [
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
            "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
            "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
            "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
            "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
            "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
            "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
            "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
        ]
    
    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """
        Preprocess image for YOLO model.
        
        Args:
            image: Input image in BGR format
            
        Returns:
            Preprocessed image tensor
        """
        # Resize
        resized = cv2.resize(image, self.input_size, interpolation=cv2.INTER_LINEAR)
        
        # BGR to RGB
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        
        # Normalize to [0, 1]
        normalized = rgb.astype(np.float32) / 255.0
        
        # HWC to CHW
        chw = np.transpose(normalized, (2, 0, 1))
        
        # Add batch dimension
        batch = np.expand_dims(chw, axis=0)
        
        return batch
    
    def postprocess(self, output: np.ndarray, orig_shape: Tuple[int, int]) -> List[Detection]:
        """
        Postprocess YOLO output.
        
        Args:
            output: Model output tensor
            orig_shape: Original image shape (height, width)
            
        Returns:
            List of Detection objects
        """
        # Calculate scale factors
        scale_x = orig_shape[1] / self.input_size[1]
        scale_y = orig_shape[0] / self.input_size[0]
        
        # Parse output (shape: [1, num_classes+4, num_detections])
        output = output[0]  # Remove batch dimension
        num_classes = len(self.class_names)
        num_detections = output.shape[1]
        
        boxes = []
        confidences = []
        class_ids = []
        
        # Transpose to [num_detections, num_classes+4]
        output = output.T
        
        for i in range(num_detections):
            detection = output[i]
            
            # Extract box coordinates
            x, y, w, h = detection[:4]
            
            # Extract class scores
            class_scores = detection[4:4+num_classes]
            
            # Find best class
            class_id = np.argmax(class_scores)
            confidence = class_scores[class_id]
            
            if confidence > self.conf_threshold:
                # Convert from center coordinates to top-left
                left = int((x - w / 2) * scale_x)
                top = int((y - h / 2) * scale_y)
                width = int(w * scale_x)
                height = int(h * scale_y)
                
                boxes.append([left, top, width, height])
                confidences.append(float(confidence))
                class_ids.append(int(class_id))
        
        # Apply NMS
        if len(boxes) > 0:
            indices = cv2.dnn.NMSBoxes(
                boxes, confidences, 
                self.conf_threshold, self.nms_threshold
            )
            
            detections = []
            if len(indices) > 0:
                for idx in indices.flatten():
                    detections.append(Detection(
                        box=boxes[idx],
                        confidence=confidences[idx],
                        class_id=class_ids[idx],
                        class_name=self.class_names[class_ids[idx]] if class_ids[idx] < len(self.class_names) else "unknown"
                    ))
            return detections
        
        return []
    
    def detect(self, image: np.ndarray) -> List[Detection]:
        """
        Run detection on an image.
        
        Args:
            image: Input image in BGR format
            
        Returns:
            List of Detection objects
        """
        orig_shape = image.shape[:2]
        
        # Preprocess
        input_tensor = self.preprocess(image)
        
        # Run inference
        output = self.session.run([self.output_name], {self.input_name: input_tensor})[0]
        
        # Postprocess
        detections = self.postprocess(output, orig_shape)
        
        return detections
    
    def draw_detections(self, image: np.ndarray, detections: List[Detection]) -> np.ndarray:
        """
        Draw detections on image.
        
        Args:
            image: Input image
            detections: List of detections
            
        Returns:
            Image with drawn detections
        """
        result = image.copy()
        
        for det in detections:
            x, y, w, h = det.box
            
            # Draw box
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Draw label
            label = f"{det.class_name}: {det.confidence:.2f}"
            (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(result, (x, y - label_h - 10), (x + label_w, y), (0, 255, 0), -1)
            cv2.putText(result, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        return result

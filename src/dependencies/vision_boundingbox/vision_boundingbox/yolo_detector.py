#!/usr/bin/env python3
"""Universal YOLO (v5/v8/v11) ONNX Runtime Detector - Robust Version"""

import numpy as np
import cv2
import onnxruntime as ort
from pathlib import Path
from typing import List, Tuple, Optional


class Detection:
    def __init__(self, box, confidence, class_id, class_name):
        self.box = box  # (x, y, w, h)
        self.confidence = confidence
        self.class_id = class_id
        self.class_name = class_name


class YOLODetector:
    def __init__(self,
                 model_path: str,
                 class_names_path: Optional[str] = None,
                 input_size: Tuple[int, int] = (640, 640),
                 conf_threshold: float = 0.5,
                 nms_threshold: float = 0.4,
                 device: str = 'CPU'):

        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold

        self.class_names = self._load_class_names(class_names_path)

        providers = []
        if device.upper() in ['CUDA', 'GPU']:
            providers.append('CUDAExecutionProvider')
        providers.append('CPUExecutionProvider')

        self.session = ort.InferenceSession(model_path, providers=providers)

        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        print(f"Model loaded: {model_path}")
        print(f"Input shape: {self.session.get_inputs()[0].shape}")
        print(f"Output shape: {self.session.get_outputs()[0].shape}")
        print(f"Execution providers: {self.session.get_providers()}")

    def _load_class_names(self, path: Optional[str]) -> List[str]:
        if path and Path(path).exists():
            with open(path, 'r') as f:
                return [line.strip() for line in f if line.strip()]

        # Default COCO
        return [
            "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
            "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
            "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
            "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
            "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
            "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
            "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
            "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
            "remote","keyboard","cell phone","microwave","oven","toaster","sink",
            "refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"
        ]

    def preprocess(self, image: np.ndarray) -> np.ndarray:
        resized = cv2.resize(image, self.input_size)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        rgb = rgb.astype(np.float32) / 255.0
        chw = np.transpose(rgb, (2, 0, 1))
        return np.expand_dims(chw, axis=0)

    def postprocess(self, output: np.ndarray, orig_shape: Tuple[int, int]) -> List[Detection]:
        if output is None or len(output) == 0:
            print("No output from model")
            return []

        output = output[0]

        if output.size == 0:
            print("Empty output tensor")
            return []

        num_classes = len(self.class_names)

        # 🔥 AUTO FORMAT DETECTION
        if len(output.shape) == 2:
            if output.shape[1] >= 6:
                detections_raw = output  # [N, 85]
            elif output.shape[0] >= 6:
                detections_raw = output.T  # [85, N]
            else:
                print("Unknown YOLO format")
                return []
        else:
            print(f"Unexpected output shape: {output.shape}")
            return []

        scale_x = orig_shape[1] / self.input_size[1]
        scale_y = orig_shape[0] / self.input_size[0]

        boxes = []
        confidences = []
        class_ids = []

        for det in detections_raw:
            if len(det) < 6:
                continue

            x, y, w, h = det[:4]

            # Format handling
            if len(det) == 6:
                confidence = float(det[4])
                class_id = int(det[5])
            else:
                obj_conf = det[4]
                class_scores = det[5:]

                if class_scores.size == 0:
                    continue

                class_id = int(np.argmax(class_scores))
                confidence = float(obj_conf * class_scores[class_id])

            if confidence > self.conf_threshold:
                left = int((x - w / 2) * scale_x)
                top = int((y - h / 2) * scale_y)
                width = int(w * scale_x)
                height = int(h * scale_y)

                boxes.append([left, top, width, height])
                confidences.append(confidence)
                class_ids.append(class_id)

        detections = []

        if len(boxes) > 0:
            indices = cv2.dnn.NMSBoxes(
                boxes, confidences,
                self.conf_threshold, self.nms_threshold
            )

            if indices is not None and len(indices) > 0:
                for idx in indices.flatten():
                    detections.append(Detection(
                        box=boxes[idx],
                        confidence=confidences[idx],
                        class_id=class_ids[idx],
                        class_name=self.class_names[class_ids[idx]]
                        if class_ids[idx] < len(self.class_names) else "unknown"
                    ))

        print(f"Detections: {len(detections)}")
        return detections

    def detect(self, image: np.ndarray) -> List[Detection]:
        orig_shape = image.shape[:2]

        try:
            input_tensor = self.preprocess(image)

            output = self.session.run(
                [self.output_name],
                {self.input_name: input_tensor}
            )[0]

            detections = self.postprocess(output, orig_shape)

        except Exception as e:
            print(f"Inference failed: {e}")
            detections = []

        print(f"Final detections count: {len(detections)}")
        return detections

    def draw_detections(self, image: np.ndarray, detections: List[Detection]) -> np.ndarray:
        result = image.copy()

        for det in detections:
            x, y, w, h = det.box

            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)

            label = f"{det.class_name}: {det.confidence:.2f}"
            (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

            cv2.rectangle(result, (x, y - lh - 10), (x + lw, y), (0, 255, 0), -1)
            cv2.putText(result, label, (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        return result


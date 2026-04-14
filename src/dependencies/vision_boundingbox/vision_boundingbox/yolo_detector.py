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

        want_gpu = device.upper() in ['CUDA', 'GPU']
        providers = []
        if want_gpu:
            available = ort.get_available_providers()
            if 'CUDAExecutionProvider' in available:
                providers.append('CUDAExecutionProvider')
            else:
                print(
                    f"WARNING: device='{device}' requested but CUDAExecutionProvider is not "
                    "available (is onnxruntime-gpu installed?). Falling back to CPU."
                )
        providers.append('CPUExecutionProvider')

        self.session = ort.InferenceSession(model_path, providers=providers)

        active = self.session.get_providers()
        if want_gpu and active[0] != 'CUDAExecutionProvider':
            print(f"WARNING: GPU requested but inference is running on {active[0]}.")

        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        print(f"Model loaded: {model_path}")
        print(f"Input shape: {self.session.get_inputs()[0].shape}")
        print(f"Output shape: {self.session.get_outputs()[0].shape}")
        print(f"Execution providers: {active}")

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
            return []

        output = output[0]  # remove batch dim

        if output.size == 0 or len(output.shape) != 2:
            return []

        # YOLOv8/v11 Ultralytics ONNX: [4+num_classes, num_anchors] — features are rows.
        # YOLOv5 ONNX:                  [num_anchors, 5+num_classes] — anchors are rows.
        # Anchors always >> features, so the smaller axis is features.
        if output.shape[0] < output.shape[1]:
            output = output.T   # → [num_anchors, 4+classes]
            yolov8 = True
        else:
            yolov8 = False

        # Vectorised confidence + class extraction
        orig_h, orig_w = orig_shape
        scale_x = orig_w / self.input_size[0]
        scale_y = orig_h / self.input_size[1]

        if yolov8:
            # [anchors, 4+classes]: no objectness, class scores start at col 4
            class_scores = output[:, 4:]          # [anchors, num_classes]
            class_ids_arr = np.argmax(class_scores, axis=1)
            confidences_arr = class_scores[np.arange(len(class_scores)), class_ids_arr]
        else:
            # [anchors, 5+classes]: objectness at col 4, class scores after
            obj_conf = output[:, 4]
            class_scores = output[:, 5:]
            class_ids_arr = np.argmax(class_scores, axis=1)
            confidences_arr = obj_conf * class_scores[np.arange(len(class_scores)), class_ids_arr]

        # Filter by confidence threshold
        mask = confidences_arr > self.conf_threshold
        if not np.any(mask):
            return []

        coords = output[mask, :4]           # [cx, cy, w, h] in model input pixels
        confidences_arr = confidences_arr[mask]
        class_ids_arr = class_ids_arr[mask]

        # Convert centre format → top-left [x, y, w, h] in original image pixels
        x1 = (coords[:, 0] - coords[:, 2] / 2) * scale_x
        y1 = (coords[:, 1] - coords[:, 3] / 2) * scale_y
        bw = coords[:, 2] * scale_x
        bh = coords[:, 3] * scale_y

        # Clip to frame boundaries (mirrors Ultralytics clip_boxes)
        x1 = np.clip(x1, 0, orig_w)
        y1 = np.clip(y1, 0, orig_h)
        bw = np.clip(bw, 0, orig_w - x1)
        bh = np.clip(bh, 0, orig_h - y1)

        boxes_arr = np.stack([x1, y1, bw, bh], axis=1).astype(int)

        # Class-aware NMS: run NMSBoxes per class so boxes of different classes
        # never suppress each other (matches Ultralytics default behaviour).
        kept: List[int] = []
        for cls in np.unique(class_ids_arr):
            cls_mask = np.where(class_ids_arr == cls)[0]
            cls_boxes = boxes_arr[cls_mask].tolist()
            cls_confs = confidences_arr[cls_mask].tolist()
            indices = cv2.dnn.NMSBoxes(cls_boxes, cls_confs, self.conf_threshold, self.nms_threshold)
            if indices is not None and len(indices) > 0:
                kept.extend(cls_mask[indices.flatten()].tolist())

        detections = []
        for idx in kept:
            cid = int(class_ids_arr[idx])
            detections.append(Detection(
                box=boxes_arr[idx].tolist(),
                confidence=float(confidences_arr[idx]),
                class_id=cid,
                class_name=self.class_names[cid] if cid < len(self.class_names) else "unknown",
            ))

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


from utils.parameters import Setting, declare_config, positive
import time
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from mira2_vision.plugin import Plugin


class Yolo(Plugin):
    requires = ()
    provides = ('detections_2d',)

    def __init__(self, node):
        super().__init__(node)
        self.config = declare_config(node, {
            'model_name': "best.onnx",
            'device': "CPU",
            'conf_threshold': Setting(0.5, lambda v: 0 <= v <= 1),
            'nms_threshold': Setting(0.4, lambda v: 0 <= v <= 1),
            'input_height': Setting(640, positive),
            'input_width': Setting(640, positive),
            'rate': Setting(15.0, positive),
        }, namespace='yolo')
        self.model = None
        self.device = self.config["device"].lower()
        if self.device == "gpu":
            self.device = "0"
        # ArUco-only deployments do not import torch or load a YOLO model.
        from ultralytics import YOLO
        model_path = node.resolve_file(self.config["model_name"], "models")
        if model_path.suffix in {".engine", ".trt"}:
            self.device = "0"
        if self.device != "cpu":
            import torch
            if not torch.cuda.is_available():
                # TensorRT engines cannot run without CUDA; everything else degrades.
                if model_path.suffix in {".engine", ".trt"}:
                    raise RuntimeError(f"{model_path.name} requires CUDA, which is unavailable")
                node.get_logger().warning("YOLO device %s requested but CUDA is unavailable; using CPU"
                                          % self.config["device"])
                self.device = "cpu"
        self.model = YOLO(str(model_path))

        self.last_run = 0.0

    def process(self, context):
        now = time.monotonic()
        if now - self.last_run < 1 / self.config['rate']:
            return
        self.last_run = now
        results = self.model.predict(context.image, conf=self.config["conf_threshold"],
                                     iou=self.config["nms_threshold"],
                                     imgsz=(self.config["input_height"], self.config["input_width"]),
                                     device=self.device, verbose=False)
        detections = Detection2DArray(header=context.header)
        if results:
            result = results[0]
            if result.boxes is not None:
                boxes = result.boxes.xyxyn.cpu().numpy()
                confidences = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy().astype(int)
                for box, confidence, class_id in zip(boxes, confidences, classes):
                    x1, y1, x2, y2 = map(float, box)
                    label = str(result.names[class_id])
                    detection = Detection2D(header=context.header, id=label)
                    detection.bbox.center.position.x = (x1 + x2) / 2
                    detection.bbox.center.position.y = (y1 + y2) / 2
                    detection.bbox.size_x, detection.bbox.size_y = x2 - x1, y2 - y1
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = label
                    hypothesis.hypothesis.score = float(confidence)
                    detection.results.append(hypothesis)
                    detections.detections.append(detection)
                if context.preview is not None:
                    context.preview = result.plot(img=context.preview)

        context.products['detections_2d'] = detections

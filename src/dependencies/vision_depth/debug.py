import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from openvino.runtime import Core

class DepthAnything:
    def __init__(self, core, device="CPU", precision="INT8"):
        package_share_dir = get_package_share_directory("vision_depth")
        if precision.upper() == "INT8":
            model_path = f"{package_share_dir}/models/depth-anything/depth-anything_int8.xml"
        else:
            model_path = f"{package_share_dir}/models/depth-anything/depth-anything_fp16.xml"

        print(f"Loading model from {model_path}")
        self.model = core.read_model(model_path)
        self.compiled_model = core.compile_model(self.model, device)
        self.input_port = self.compiled_model.input(0)
        self.output_port = self.compiled_model.output(0)

    def preprocess(self, frame):
        # Resize to 518x518
        resized = cv2.resize(frame, (518, 518), interpolation=cv2.INTER_CUBIC)
        # Convert BGR -> RGB
        resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        # Normalize
        resized = resized.astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        resized = (resized - mean) / std
        # HWC -> CHW
        chw = np.transpose(resized, (2, 0, 1))
        # Add batch dimension
        return np.expand_dims(chw, axis=0)

    def process_frame(self, frame):
        input_tensor = self.preprocess(frame)
        result = self.compiled_model.infer_new_request({self.input_port.any_name: input_tensor})
        output_tensor = result[self.output_port.any_name]

        depth_map = np.squeeze(output_tensor)
        depth_vis = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_MAGMA)
        return depth_vis

def main():
    rclpy.init()
    node = Node("depth_anything_node")

    core = Core()
    depth_anything = DepthAnything(core, device=node.declare_parameter("device", "CPU").value,
                                   precision=node.declare_parameter("precision", "INT8").value)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        node.get_logger().error("Failed to open camera")
        return

    frame_count = 0
    fps = 0.0
    last_time = cv2.getTickCount()

    while rclpy.ok():
        ret, frame = cap.read()
        if not ret:
            node.get_logger().error("Failed to read frame from camera")
            break

        depth_vis = depth_anything.process_frame(frame)

        # FPS calculation
        frame_count += 1
        current_time = cv2.getTickCount()
        elapsed = (current_time - last_time) / cv2.getTickFrequency()
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            last_time = current_time

        cv2.putText(depth_vis, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (0, 255, 0), 2)

        cv2.imshow("Depth Anything V2", depth_vis)
        if cv2.waitKey(1) == 27:  # ESC key
            break

    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

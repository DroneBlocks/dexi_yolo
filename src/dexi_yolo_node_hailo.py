#!/usr/bin/env python3
"""
YOLO detection ROS2 node using Hailo 8L AI accelerator
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from dexi_interfaces.msg import YoloDetection as YoloDetectionMsg, YoloDetectionArray
import cv2
import numpy as np
import time
import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from hailo_platform import (
    HEF, VDevice, HailoStreamInterface, InferVStreams,
    ConfigureParams, InputVStreamParams, OutputVStreamParams, FormatType
)


class DexiYoloHailoNode(Node):
    """ROS2 node for YOLO object detection using Hailo 8L"""

    def __init__(self):
        super().__init__('dexi_yolo_hailo_node')

        # Parameters
        pkg_share = get_package_share_directory('dexi_yolo')
        default_hef = os.path.join(pkg_share, 'models', 'best_optimized.hef')
        self.declare_parameter('hef_path', default_hef)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('detection_frequency', 10.0)  # Hz
        self.declare_parameter('class_names', 'car,motorcycle,truck,bird,cat,dog')

        hef_path = self.get_parameter('hef_path').value
        self.hef_path = hef_path if hef_path else default_hef
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.detection_frequency = self.get_parameter('detection_frequency').value
        class_names_str = self.get_parameter('class_names').value
        self.class_names = [name.strip() for name in class_names_str.split(',')]
        self.get_logger().info(f"Classes ({len(self.class_names)}): {self.class_names}")

        # Initialize Hailo
        self.get_logger().info(f"Loading HEF model: {self.hef_path}")
        self.hef = HEF(self.hef_path)
        self.target = VDevice()
        configure_params = ConfigureParams.create_from_hef(
            self.hef, interface=HailoStreamInterface.PCIe
        )
        self.network_group = self.target.configure(self.hef, configure_params)[0]
        self.input_vstreams_params = InputVStreamParams.make(
            self.network_group, format_type=FormatType.UINT8
        )
        self.output_vstreams_params = OutputVStreamParams.make(
            self.network_group, format_type=FormatType.FLOAT32
        )
        self.input_info = self.hef.get_input_vstream_infos()[0]
        self.input_h, self.input_w, self.input_c = self.input_info.shape

        # Activate network group and open inference pipeline (kept open)
        self.network_group_ctx = self.network_group.activate(
            network_group_params=self.network_group.create_params()
        )
        self.network_group_ctx.__enter__()

        self.pipeline_ctx = InferVStreams(
            self.network_group, self.input_vstreams_params, self.output_vstreams_params
        )
        self.pipeline = self.pipeline_ctx.__enter__()

        self.get_logger().info(
            f"Hailo ready: input {self.input_h}x{self.input_w}x{self.input_c}"
        )

        # Warm up
        dummy = np.zeros((self.input_h, self.input_w, self.input_c), dtype=np.uint8)
        self.pipeline.infer({self.input_info.name: np.expand_dims(dummy, 0)})
        self.get_logger().info("Warm-up inference complete")

        # Publishers
        self.detection_pub = self.create_publisher(YoloDetectionArray, '/yolo_detections', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            CompressedImage, '/cam0/image_raw/compressed', self.image_callback, 10
        )

        # Rate limiting
        self.last_detection_time = 0.0
        self.min_detection_interval = 1.0 / self.detection_frequency

        # Stats
        self.frame_count = 0
        self.total_inference_ms = 0.0

        self.get_logger().info(
            f"Node ready — subscribing /cam0/image_raw/compressed, "
            f"publishing /yolo_detections @ up to {self.detection_frequency} Hz, "
            f"conf >= {self.confidence_threshold}"
        )

    def image_callback(self, msg: CompressedImage):
        now = time.time()
        if now - self.last_detection_time < self.min_detection_interval:
            return
        self.last_detection_time = now

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image is None:
                return

            # Preprocess: resize to model input, keep as uint8 (Hailo expects uint8)
            resized = cv2.resize(image, (self.input_w, self.input_h))
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

            # Infer
            t0 = time.perf_counter()
            results = self.pipeline.infer(
                {self.input_info.name: np.expand_dims(rgb, 0)}
            )
            inference_ms = (time.perf_counter() - t0) * 1000

            self.frame_count += 1
            self.total_inference_ms += inference_ms

            # Parse NMS output — list of 80 arrays, each (N_i, 5) per class
            output_name = list(results.keys())[0]
            raw = results[output_name]
            # Unwrap batch dimension if present
            if isinstance(raw, list) and len(raw) > 0 and isinstance(raw[0], list):
                raw = raw[0]

            detections = self._parse_detections(raw, image.shape[:2])

            if detections:
                self._publish(detections, msg.header)

            # Log periodically
            if self.frame_count % 30 == 0:
                avg_ms = self.total_inference_ms / self.frame_count
                det_str = ""
                if detections:
                    det_str = " | " + ", ".join(
                        f"{d.class_name}({d.confidence:.2f})" for d in detections[:3]
                    )
                self.get_logger().info(
                    f"Frame {self.frame_count}: {avg_ms:.1f}ms avg "
                    f"({1000/avg_ms:.0f} FPS){det_str}"
                )

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def _parse_detections(self, raw_output, original_shape):
        """Parse Hailo NMS output — list of per-class arrays from yolov8_nms_postprocess.
        Each per-class array has shape (5, N_i) where 5 = [y_min, x_min, y_max, x_max, score]
        and N_i is the number of detections for that class."""
        detections = []

        # Log output structure on first frame for debugging
        if self.frame_count == 1:
            self.get_logger().info(f"NMS output: {len(raw_output)} classes")
            for i, class_dets in enumerate(raw_output):
                arr = np.array(class_dets)
                name = self.class_names[i] if i < len(self.class_names) else f"class_{i}"
                self.get_logger().info(f"  [{i}] {name}: shape={arr.shape}, dtype={arr.dtype}")
                if arr.size > 0:
                    self.get_logger().info(f"       sample: {arr.flat[:5]}")

        for cls_idx, class_dets in enumerate(raw_output):
            arr = np.array(class_dets)
            if arr.size == 0:
                continue

            # Determine layout: (5, N) or (N, 5)
            if arr.ndim == 2 and arr.shape[0] == 5:
                # (5, N) — rows are [y_min, x_min, y_max, x_max, score], columns are detections
                num_dets = arr.shape[1]
                for det_idx in range(num_dets):
                    conf = float(arr[4, det_idx])
                    if conf < self.confidence_threshold:
                        continue
                    y_min = float(arr[0, det_idx])
                    x_min = float(arr[1, det_idx])
                    y_max = float(arr[2, det_idx])
                    x_max = float(arr[3, det_idx])

                    bbox = [max(0.0, x_min), max(0.0, y_min),
                            min(1.0, x_max), min(1.0, y_max)]
                    class_name = self.class_names[cls_idx] if cls_idx < len(self.class_names) else f"class_{cls_idx}"
                    det_msg = YoloDetectionMsg()
                    det_msg.class_name = class_name
                    det_msg.confidence = conf
                    det_msg.bbox = bbox
                    detections.append(det_msg)

            elif arr.ndim == 2 and arr.shape[1] == 5:
                # (N, 5) — rows are detections, columns are [y_min, x_min, y_max, x_max, score]
                for det_idx in range(arr.shape[0]):
                    conf = float(arr[det_idx, 4])
                    if conf < self.confidence_threshold:
                        continue
                    y_min = float(arr[det_idx, 0])
                    x_min = float(arr[det_idx, 1])
                    y_max = float(arr[det_idx, 2])
                    x_max = float(arr[det_idx, 3])

                    bbox = [max(0.0, x_min), max(0.0, y_min),
                            min(1.0, x_max), min(1.0, y_max)]
                    class_name = self.class_names[cls_idx] if cls_idx < len(self.class_names) else f"class_{cls_idx}"
                    det_msg = YoloDetectionMsg()
                    det_msg.class_name = class_name
                    det_msg.confidence = conf
                    det_msg.bbox = bbox
                    detections.append(det_msg)

        return detections

    def _publish(self, detections, header):
        msg = YoloDetectionArray()
        msg.header = header
        msg.timestamp = time.time()
        msg.detections = detections
        self.detection_pub.publish(msg)

    def destroy_node(self):
        try:
            self.pipeline_ctx.__exit__(None, None, None)
            self.network_group_ctx.__exit__(None, None, None)
            self.target.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DexiYoloHailoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node is not None:
            if node.frame_count > 0:
                avg = node.total_inference_ms / node.frame_count
                print(f"\nHailo stats: {node.frame_count} frames, {avg:.1f}ms avg ({1000/avg:.0f} FPS)")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

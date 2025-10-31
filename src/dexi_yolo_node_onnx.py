#!/usr/bin/env python3
"""
YOLO detection ROS2 node using ONNX Runtime for optimized performance
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from geometry_msgs.msg import Point32
from std_msgs.msg import String, Float32MultiArray
from dexi_interfaces.msg import YoloDetection as YoloDetectionMsg, YoloDetectionArray
import cv2
import numpy as np
import onnxruntime as ort
import time
from typing import List, Tuple
import json
import os
from ament_index_python.packages import get_package_share_directory

class YoloDetection:
    """Represents a single YOLO detection"""
    def __init__(self, class_name: str, confidence: float, bbox: List[float]):
        self.class_name = class_name
        self.confidence = confidence
        self.bbox = bbox  # [x1, y1, x2, y2] - normalized coordinates
    
    def to_dict(self):
        return {
            'class_name': self.class_name,
            'confidence': self.confidence,
            'bbox': self.bbox
        }

class DexiYoloOnnxNode(Node):
    """ROS2 node for YOLO object detection using ONNX Runtime"""
    
    def __init__(self):
        super().__init__('dexi_yolo_onnx_node')
        
        # Parameters
        self.declare_parameter('model_path', 'models/best_optimized.onnx')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('detection_frequency', 1.0)  # Hz
        self.declare_parameter('input_size', 320)  # Model input size (320x320 training)
        self.declare_parameter('num_threads', 2)   # Limit CPU threads
        self.declare_parameter('nms_threshold', 0.4)  # IoU threshold for NMS
        self.declare_parameter('use_letterbox', True)  # Use letterbox preprocessing (preserves aspect ratio)
        self.declare_parameter('verbose_logging', False)  # Enable verbose detection logging
        self.declare_parameter('max_detections', 10)  # Maximum detections to keep (limits processing)
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.detection_frequency = self.get_parameter('detection_frequency').value
        self.input_size = self.get_parameter('input_size').value
        self.num_threads = self.get_parameter('num_threads').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.use_letterbox = self.get_parameter('use_letterbox').value
        self.verbose_logging = self.get_parameter('verbose_logging').value
        self.max_detections = self.get_parameter('max_detections').value
        
        # Resolve model path to absolute path
        self.model_path = self._resolve_model_path(self.model_path)
        
        # Pre-allocate arrays for efficiency
        self.input_tensor = np.zeros((1, 3, self.input_size, self.input_size), dtype=np.float32)
        self.resized_buffer = np.zeros((self.input_size, self.input_size, 3), dtype=np.uint8)
        
        # Initialize ONNX Runtime session with CPU optimization
        self.get_logger().info(f"Loading ONNX model from {self.model_path}...")
        try:
            # Optimized session options for Pi CM4
            sess_options = ort.SessionOptions()
            sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            sess_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            
            # Limit threads to prevent CPU oversubscription
            sess_options.intra_op_num_threads = self.num_threads
            sess_options.inter_op_num_threads = 1
            
            # Memory optimizations
            sess_options.enable_mem_pattern = False  # Disable for lower memory
            sess_options.enable_cpu_mem_arena = False  # Reduce memory fragmentation
            
            # CPU-specific provider with optimizations
            providers = [('CPUExecutionProvider', {
                'use_arena': False,  # Reduce memory overhead
                'enable_cpu_mem_arena': False
            })]
            
            self.session = ort.InferenceSession(
                self.model_path, 
                sess_options=sess_options,
                providers=providers
            )
            
            # Get model input/output info
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            
            # Custom trained class names (exact order from training)
            self.class_names = [
                'car', 'motorcycle', 'truck', 'bird', 'cat', 'dog'
            ]
            
            self.get_logger().info("Optimized ONNX model loaded successfully!")
            self.get_logger().info(f"Input: {self.input_name}, Output: {self.output_name}")
            self.get_logger().info(f"CPU threads: intra={self.num_threads}, inter=1")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX model: {e}")
            raise
        
        # Publishers
        self.detection_pub = self.create_publisher(
            YoloDetectionArray, 
            '/yolo_detections', 
            10
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/cam0/image_raw/compressed',
            self.image_callback,
            10
        )
        
        # Timer for detection frequency control
        self.last_detection_time = 0.0
        self.min_detection_interval = 1.0 / self.detection_frequency
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        self.total_inference_time = 0.0
        self.total_preprocess_time = 0.0
        self.total_postprocess_time = 0.0
        
        self.get_logger().info("DEXI YOLO ONNX node initialized successfully!")
        self.get_logger().info(f"Subscribing to: /cam0/image_raw/compressed")
        self.get_logger().info(f"Publishing to: /yolo_detections")
        self.get_logger().info(f"Detection frequency: {self.detection_frequency} Hz")
        self.get_logger().info(f"Confidence threshold: {self.confidence_threshold}")
        self.get_logger().info(f"NMS threshold: {self.nms_threshold}")
        self.get_logger().info(f"Input size: {self.input_size}x{self.input_size}")
        self.get_logger().info(f"Preprocessing: {'Letterbox' if self.use_letterbox else 'Simple Resize'}")
        self.get_logger().info(f"CPU threads limited to: {self.num_threads}")
    
    def _resolve_model_path(self, model_path: str) -> str:
        """Resolve model path to absolute path, handling package-relative paths"""
        # If it's already an absolute path, return as is
        if os.path.isabs(model_path):
            return model_path
        
        # If it's a relative path starting with 'models/', resolve from package share directory
        if model_path.startswith('models/'):
            try:
                package_share_dir = get_package_share_directory('dexi_yolo')
                resolved_path = os.path.join(package_share_dir, model_path)
                self.get_logger().info(f"Resolved model path: {resolved_path}")
                return resolved_path
            except Exception as e:
                self.get_logger().warn(f"Could not resolve package path, using relative: {e}")
                return model_path
        
        # For other relative paths, return as is (user can specify absolute paths)
        return model_path
    
    def preprocess_image_optimized(self, image: np.ndarray) -> np.ndarray:
        """Optimized image preprocessing with minimal memory allocations"""
        if self.use_letterbox:
            # Letterbox preprocessing: preserves aspect ratio with padding
            orig_h, orig_w = image.shape[:2]

            # Calculate scale to fit inside square while preserving aspect ratio
            scale = min(self.input_size / orig_w, self.input_size / orig_h)
            new_w = int(orig_w * scale)
            new_h = int(orig_h * scale)

            # Resize with aspect ratio preserved
            resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

            # Create padded image (letterbox) with gray padding (114 = dark gray)
            padded = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)

            # Calculate padding offsets (center the image)
            top = (self.input_size - new_h) // 2
            left = (self.input_size - new_w) // 2

            padded[top:top+new_h, left:left+new_w] = resized

            # Convert BGR to RGB
            rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        else:
            # Simple resize preprocessing (distorts if aspect ratio doesn't match)
            # Use pre-allocated buffer for resize
            cv2.resize(image, (self.input_size, self.input_size), dst=self.resized_buffer, interpolation=cv2.INTER_LINEAR)

            # Convert BGR to RGB in-place
            cv2.cvtColor(self.resized_buffer, cv2.COLOR_BGR2RGB, dst=self.resized_buffer)
            rgb = self.resized_buffer

        # FIXED: Proper normalization and convert HWC to CHW format
        normalized = rgb.astype(np.float32) / 255.0
        self.input_tensor[0] = normalized.transpose(2, 0, 1)  # HWC -> CHW

        return self.input_tensor
    
    def calculate_iou(self, box1: List[float], box2: List[float]) -> float:
        """Calculate Intersection over Union (IoU) between two bounding boxes"""
        x1_min, y1_min, x1_max, y1_max = box1
        x2_min, y2_min, x2_max, y2_max = box2
        
        # Calculate intersection area
        inter_x_min = max(x1_min, x2_min)
        inter_y_min = max(y1_min, y2_min)
        inter_x_max = min(x1_max, x2_max)
        inter_y_max = min(y1_max, y2_max)
        
        if inter_x_max <= inter_x_min or inter_y_max <= inter_y_min:
            return 0.0
        
        inter_area = (inter_x_max - inter_x_min) * (inter_y_max - inter_y_min)
        
        # Calculate union area
        box1_area = (x1_max - x1_min) * (y1_max - y1_min)
        box2_area = (x2_max - x2_min) * (y2_max - y2_min)
        union_area = box1_area + box2_area - inter_area
        
        return inter_area / union_area if union_area > 0 else 0.0
    
    def apply_nms(self, detections: List[YoloDetection]) -> List[YoloDetection]:
        """Apply Non-Maximum Suppression to remove duplicate detections"""
        if len(detections) <= 1:
            return detections
        
        # Sort detections by confidence (highest first)
        detections.sort(key=lambda x: x.confidence, reverse=True)
        
        # Group detections by class
        class_groups = {}
        for detection in detections:
            if detection.class_name not in class_groups:
                class_groups[detection.class_name] = []
            class_groups[detection.class_name].append(detection)
        
        # Apply NMS per class
        final_detections = []
        for class_name, class_detections in class_groups.items():
            keep = []
            
            for i, detection in enumerate(class_detections):
                should_keep = True
                
                # Check against all previously kept detections
                for kept_detection in keep:
                    iou = self.calculate_iou(detection.bbox, kept_detection.bbox)
                    if iou > self.nms_threshold:
                        should_keep = False
                        break
                
                if should_keep:
                    keep.append(detection)
            
            final_detections.extend(keep)
        
        return final_detections
    
    def postprocess_detections_optimized(self, output: np.ndarray, original_shape: Tuple[int, int]) -> List[YoloDetection]:
        """Optimized postprocessing with NMS and reduced memory allocations"""
        detections = []

        # YOLOv8 output shape: (1, 10, 2100) where 10 = 4 bbox coords + 6 classes
        output = output[0].T  # Remove batch dimension and transpose to (2100, 10)

        # Extract bbox and class scores
        bbox_coords = output[:, :4]
        class_scores = output[:, 4:]

        # Apply sigmoid only if scores are raw logits (not already probabilities)
        if class_scores.min() < 0 or class_scores.max() > 1.5:
            class_scores = 1 / (1 + np.exp(-class_scores))

        # Pre-filter by confidence to reduce processing
        max_scores = np.max(class_scores, axis=1)
        valid_indices = max_scores >= self.confidence_threshold

        if not np.any(valid_indices):
            return detections

        # Process only valid detections
        valid_bbox = bbox_coords[valid_indices]
        valid_scores = class_scores[valid_indices]
        orig_h, orig_w = original_shape
        scale_x = orig_w / self.input_size
        scale_y = orig_h / self.input_size

        for idx in range(len(valid_bbox)):
            # First 4 values are bbox coordinates (cx, cy, w, h)
            cx, cy, w, h = valid_bbox[idx]

            # Class scores (already sigmoid-activated)
            scores = valid_scores[idx]

            # Find class with highest confidence
            max_score_idx = np.argmax(scores)
            confidence = scores[max_score_idx]

            # Convert from center format to corner format
            x1 = max(0, (cx - w / 2) * scale_x / orig_w)
            y1 = max(0, (cy - h / 2) * scale_y / orig_h)
            x2 = min(1, (cx + w / 2) * scale_x / orig_w)
            y2 = min(1, (cy + h / 2) * scale_y / orig_h)

            class_name = self.class_names[max_score_idx]
            bbox = [float(x1), float(y1), float(x2), float(y2)]

            detection_obj = YoloDetection(class_name, float(confidence), bbox)
            detections.append(detection_obj)
        
        # Apply Non-Maximum Suppression to remove duplicates
        if len(detections) > 1:
            detections = self.apply_nms(detections)

        # Limit to max detections (keep highest confidence)
        if len(detections) > self.max_detections:
            detections.sort(key=lambda x: x.confidence, reverse=True)
            detections = detections[:self.max_detections]

        return detections
    
    def image_callback(self, msg: CompressedImage):
        """Callback for incoming compressed images"""
        current_time = time.time()
        
        # Check if enough time has passed since last detection
        if current_time - self.last_detection_time < self.min_detection_interval:
            return
        
        self.frame_count += 1
        
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is None:
                self.get_logger().warn("Failed to decode compressed image")
                return
            
            original_shape = image.shape[:2]  # (height, width)
            
            # Run optimized detection
            detections = self.run_detection_optimized(image, original_shape)
            
            if detections:
                self.publish_detections(detections, msg.header)
                self.detection_count += 1

                # Log detection results with detailed timing (only if verbose)
                if self.verbose_logging:
                    detection_info = [f"{d.class_name}({d.confidence:.2f})" for d in detections]
                    avg_inference = self.total_inference_time / self.frame_count * 1000
                    avg_preprocess = self.total_preprocess_time / self.frame_count * 1000
                    avg_postprocess = self.total_postprocess_time / self.frame_count * 1000

                    self.get_logger().info(
                        f"Frame {self.frame_count}: {', '.join(detection_info)} "
                        f"(inf: {avg_inference:.1f}ms, pre: {avg_preprocess:.1f}ms, post: {avg_postprocess:.1f}ms)"
                    )
            
            self.last_detection_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def run_detection_optimized(self, image: np.ndarray, original_shape: Tuple[int, int]) -> List[YoloDetection]:
        """Run optimized ONNX inference on the image"""
        try:
            # Optimized preprocessing
            start_time = time.time()
            input_tensor = self.preprocess_image_optimized(image)
            preprocess_time = time.time() - start_time
            self.total_preprocess_time += preprocess_time
            
            # Run inference
            start_time = time.time()
            outputs = self.session.run([self.output_name], {self.input_name: input_tensor})
            inference_time = time.time() - start_time
            self.total_inference_time += inference_time
            
            # Optimized postprocessing
            start_time = time.time()
            detections = self.postprocess_detections_optimized(outputs[0], original_shape)
            postprocess_time = time.time() - start_time
            self.total_postprocess_time += postprocess_time
            
            return detections
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")
            return []
    
    def publish_detections(self, detections: List[YoloDetection], header: Header):
        """Publish detection results using custom message type"""
        try:
            # Create message
            msg = YoloDetectionArray()
            msg.header = header
            msg.timestamp = time.time()
            
            # Convert YoloDetection objects to message format
            for detection in detections:
                detection_msg = YoloDetectionMsg()
                detection_msg.class_name = detection.class_name
                detection_msg.confidence = detection.confidence
                detection_msg.bbox = detection.bbox
                msg.detections.append(detection_msg)
            
            # Publish
            self.detection_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing detections: {e}")
    
    def get_statistics(self):
        """Get node statistics"""
        frames = max(self.frame_count, 1)
        avg_inference_time = self.total_inference_time / frames
        avg_preprocess_time = self.total_preprocess_time / frames
        avg_postprocess_time = self.total_postprocess_time / frames
        
        return {
            'frames_processed': self.frame_count,
            'detections_made': self.detection_count,
            'detection_rate': self.detection_count / frames,
            'avg_inference_time_ms': avg_inference_time * 1000,
            'avg_preprocess_time_ms': avg_preprocess_time * 1000,
            'avg_postprocess_time_ms': avg_postprocess_time * 1000,
            'total_time': self.total_inference_time + self.total_preprocess_time + self.total_postprocess_time
        }

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = DexiYoloOnnxNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node is not None:
            # Print final statistics
            stats = node.get_statistics()
            print(f"\nFinal Statistics (ONNX):")
            print(f"Frames processed: {stats['frames_processed']}")
            print(f"Detections made: {stats['detections_made']}")
            print(f"Detection rate: {stats['detection_rate']:.2f}")
            print(f"Average inference time: {stats['avg_inference_time_ms']:.1f}ms")
            print(f"Average preprocess time: {stats['avg_preprocess_time_ms']:.1f}ms")
            print(f"Average postprocess time: {stats['avg_postprocess_time_ms']:.1f}ms")
            print(f"Total processing time: {stats['total_time']:.2f}s")

            node.destroy_node()

        # Only shutdown if rclpy context is still valid
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

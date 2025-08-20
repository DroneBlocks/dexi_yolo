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
        self.declare_parameter('model_path', 'models/yolov8n.onnx')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('detection_frequency', 1.0)  # Hz
        self.declare_parameter('input_size', 320)  # Model input size
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.detection_frequency = self.get_parameter('detection_frequency').value
        self.input_size = self.get_parameter('input_size').value
        
        # Resolve model path to absolute path
        self.model_path = self._resolve_model_path(self.model_path)
        
        # Initialize ONNX Runtime session
        self.get_logger().info(f"Loading ONNX model from {self.model_path}...")
        try:
            # Configure ONNX Runtime for optimization
            providers = ['CPUExecutionProvider']
            if ort.get_device() == 'GPU':
                providers.insert(0, 'CUDAExecutionProvider')
            
            sess_options = ort.SessionOptions()
            sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            sess_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            
            self.session = ort.InferenceSession(
                self.model_path, 
                sess_options=sess_options,
                providers=providers
            )
            
            # Get model input/output info
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            
            # COCO class names (YOLOv8 default)
            self.class_names = [
                'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
                'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
                'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
                'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
                'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
                'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
                'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake',
                'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
                'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
                'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
                'toothbrush'
            ]
            
            self.get_logger().info("ONNX model loaded successfully!")
            self.get_logger().info(f"Input: {self.input_name}, Output: {self.output_name}")
            self.get_logger().info(f"Available providers: {self.session.get_providers()}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX model: {e}")
            raise
        
        # Publishers
        self.detection_pub = self.create_publisher(
            String, 
            '/yolo_detections_onnx', 
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
        
        self.get_logger().info("Dexi YOLO ONNX node initialized successfully!")
        self.get_logger().info(f"Subscribing to: /cam0/image_raw/compressed")
        self.get_logger().info(f"Publishing to: /yolo_detections_onnx")
        self.get_logger().info(f"Detection frequency: {self.detection_frequency} Hz")
        self.get_logger().info(f"Confidence threshold: {self.confidence_threshold}")
        self.get_logger().info(f"Input size: {self.input_size}x{self.input_size}")
    
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
    
    def preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """Preprocess image for ONNX model input"""
        # Resize image to model input size
        resized = cv2.resize(image, (self.input_size, self.input_size))
        
        # Convert BGR to RGB
        rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        
        # Normalize to [0,1] and convert to NCHW format
        input_tensor = rgb_image.astype(np.float32) / 255.0
        input_tensor = np.transpose(input_tensor, (2, 0, 1))  # HWC to CHW
        input_tensor = np.expand_dims(input_tensor, axis=0)   # Add batch dimension
        
        return input_tensor
    
    def postprocess_detections(self, output: np.ndarray, original_shape: Tuple[int, int]) -> List[YoloDetection]:
        """Postprocess ONNX model output to extract detections"""
        detections = []
        
        # YOLOv8 output shape: (1, 84, 2100) where 84 = 4 bbox coords + 80 classes
        output = output[0]  # Remove batch dimension: (84, 2100)
        output = output.T   # Transpose to (2100, 84)
        
        # Extract bounding boxes and scores
        for detection in output:
            # First 4 values are bbox coordinates (cx, cy, w, h)
            cx, cy, w, h = detection[:4]
            
            # Remaining 80 values are class scores
            class_scores = detection[4:]
            
            # Find class with highest confidence
            max_score_idx = np.argmax(class_scores)
            confidence = class_scores[max_score_idx]
            
            # Filter by confidence threshold
            if confidence < self.confidence_threshold:
                continue
            
            # Convert from center format to corner format
            x1 = cx - w / 2
            y1 = cy - h / 2
            x2 = cx + w / 2
            y2 = cy + h / 2
            
            # Normalize coordinates to original image size
            orig_h, orig_w = original_shape
            scale_x = orig_w / self.input_size
            scale_y = orig_h / self.input_size
            
            x1 = max(0, x1 * scale_x / orig_w)
            y1 = max(0, y1 * scale_y / orig_h)
            x2 = min(1, x2 * scale_x / orig_w)
            y2 = min(1, y2 * scale_y / orig_h)
            
            class_name = self.class_names[max_score_idx]
            bbox = [float(x1), float(y1), float(x2), float(y2)]
            
            detection_obj = YoloDetection(class_name, float(confidence), bbox)
            detections.append(detection_obj)
        
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
            
            # Run ONNX inference
            detections = self.run_detection(image, original_shape)
            
            if detections:
                self.publish_detections(detections, msg.header)
                self.detection_count += 1
                
                # Log detection results
                detection_info = [f"{d.class_name}({d.confidence:.2f})" for d in detections]
                avg_inference_time = self.total_inference_time / self.frame_count
                self.get_logger().info(f"Frame {self.frame_count}: {', '.join(detection_info)} (avg: {avg_inference_time*1000:.1f}ms)")
            else:
                self.get_logger().debug(f"Frame {self.frame_count}: No objects detected")
            
            self.last_detection_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def run_detection(self, image: np.ndarray, original_shape: Tuple[int, int]) -> List[YoloDetection]:
        """Run ONNX inference on the image"""
        try:
            # Preprocess image
            input_tensor = self.preprocess_image(image)
            
            # Run inference
            start_time = time.time()
            outputs = self.session.run([self.output_name], {self.input_name: input_tensor})
            inference_time = time.time() - start_time
            self.total_inference_time += inference_time
            
            # Postprocess outputs
            detections = self.postprocess_detections(outputs[0], original_shape)
            
            return detections
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")
            return []
    
    def publish_detections(self, detections: List[YoloDetection], header: Header):
        """Publish detection results as JSON string"""
        try:
            # Convert detections to JSON-serializable format
            detection_data = {
                'header': {
                    'stamp': {
                        'sec': header.stamp.sec,
                        'nanosec': header.stamp.nanosec
                    },
                    'frame_id': header.frame_id
                },
                'detections': [d.to_dict() for d in detections],
                'timestamp': time.time(),
                'engine': 'onnx'
            }
            
            # Create message
            msg = String()
            msg.data = json.dumps(detection_data)
            
            # Publish
            self.detection_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing detections: {e}")
    
    def get_statistics(self):
        """Get node statistics"""
        avg_inference_time = self.total_inference_time / max(self.frame_count, 1)
        return {
            'frames_processed': self.frame_count,
            'detections_made': self.detection_count,
            'detection_rate': self.detection_count / max(self.frame_count, 1),
            'avg_inference_time_ms': avg_inference_time * 1000,
            'total_inference_time': self.total_inference_time
        }

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DexiYoloOnnxNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            # Print final statistics
            stats = node.get_statistics()
            print(f"\nFinal Statistics (ONNX):")
            print(f"Frames processed: {stats['frames_processed']}")
            print(f"Detections made: {stats['detections_made']}")
            print(f"Detection rate: {stats['detection_rate']:.2f}")
            print(f"Average inference time: {stats['avg_inference_time_ms']:.1f}ms")
            print(f"Total inference time: {stats['total_inference_time']:.2f}s")
            
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
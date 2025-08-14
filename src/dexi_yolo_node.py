#!/usr/bin/env python3
"""
YOLO detection ROS2 node that subscribes to compressed images and publishes detections
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from geometry_msgs.msg import Point32
from std_msgs.msg import String, Float32MultiArray
import cv2
import numpy as np
from ultralytics import YOLO
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

class DexiYoloNode(Node):
    """ROS2 node for YOLO object detection"""
    
    def __init__(self):
        super().__init__('dexi_yolo_node')
        
        # Parameters
        self.declare_parameter('model_path', 'models/yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('detection_frequency', 1.0)  # Hz
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.detection_frequency = self.get_parameter('detection_frequency').value
        
        # Resolve model path to absolute path
        self.model_path = self._resolve_model_path(self.model_path)
        
        # Initialize YOLO model
        self.get_logger().info(f"Loading YOLO model from {self.model_path}...")
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info("YOLO model loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise
        
        # Publishers
        self.detection_pub = self.create_publisher(
            String, 
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
        
        self.get_logger().info("Dexi YOLO node initialized successfully!")
        self.get_logger().info(f"Subscribing to: /cam0/image_raw/compressed")
        self.get_logger().info(f"Publishing to: /yolo_detections")
        self.get_logger().info(f"Detection frequency: {self.detection_frequency} Hz")
        self.get_logger().info(f"Confidence threshold: {self.confidence_threshold}")
    
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
            
            # Convert BGR to RGB for YOLO
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Run YOLO detection
            detections = self.run_detection(image_rgb)
            
            if detections:
                self.publish_detections(detections, msg.header)
                self.detection_count += 1
                
                # Log detection results
                detection_info = [f"{d.class_name}({d.confidence:.2f})" for d in detections]
                self.get_logger().info(f"Frame {self.frame_count}: {', '.join(detection_info)}")
            else:
                self.get_logger().debug(f"Frame {self.frame_count}: No objects detected")
            
            self.last_detection_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def run_detection(self, image: np.ndarray) -> List[YoloDetection]:
        """Run YOLO detection on the image"""
        try:
            results = self.model(image, conf=self.confidence_threshold, verbose=False)
            
            detections = []
            if results[0].boxes is not None and len(results[0].boxes) > 0:
                height, width = image.shape[:2]
                
                for box in results[0].boxes:
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    class_name = results[0].names[cls]
                    
                    # Get bounding box coordinates (normalized)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    bbox = [float(x1/width), float(y1/height), float(x2/width), float(y2/height)]
                    
                    detection = YoloDetection(class_name, conf, bbox)
                    detections.append(detection)
            
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
                'timestamp': time.time()
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
        return {
            'frames_processed': self.frame_count,
            'detections_made': self.detection_count,
            'detection_rate': self.detection_count / max(self.frame_count, 1)
        }

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DexiYoloNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            # Print final statistics
            stats = node.get_statistics()
            print(f"\nFinal Statistics:")
            print(f"Frames processed: {stats['frames_processed']}")
            print(f"Detections made: {stats['detections_made']}")
            print(f"Detection rate: {stats['detection_rate']:.2f}")
            
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

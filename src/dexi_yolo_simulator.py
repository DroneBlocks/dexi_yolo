#!/usr/bin/env python3
"""
YOLO detection simulator for desktop development and testing
Publishes realistic detection data without requiring Pi CM4 hardware
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import random
import math
from typing import List, Dict, Tuple

class YoloDetectionSimulator(Node):
    """Simulator node that publishes realistic YOLO detection data"""
    
    def __init__(self):
        super().__init__('dexi_yolo_simulator')
        
        # Parameters
        self.declare_parameter('detection_frequency', 1.0)  # Hz
        self.declare_parameter('scenario', 'mixed')  # traffic, indoor, outdoor, mixed
        self.declare_parameter('detection_probability', 0.7)  # Chance of detecting objects per frame
        self.declare_parameter('max_objects_per_frame', 3)
        self.declare_parameter('enable_movement', True)  # Objects move between frames
        self.declare_parameter('confidence_range', [0.5, 0.95])  # Min/max confidence
        
        # Get parameters
        self.detection_frequency = self.get_parameter('detection_frequency').value
        self.scenario = self.get_parameter('scenario').value
        self.detection_probability = self.get_parameter('detection_probability').value
        self.max_objects = self.get_parameter('max_objects_per_frame').value
        self.enable_movement = self.get_parameter('enable_movement').value
        self.confidence_range = self.get_parameter('confidence_range').value
        
        # Object scenarios with realistic class distributions
        self.scenarios = {
            'traffic': {
                'classes': ['car', 'truck', 'bus', 'motorcycle', 'bicycle', 'person', 'traffic light', 'stop sign'],
                'weights': [0.4, 0.15, 0.05, 0.1, 0.1, 0.15, 0.03, 0.02]
            },
            'indoor': {
                'classes': ['person', 'chair', 'dining table', 'laptop', 'tv', 'book', 'cup', 'cell phone'],
                'weights': [0.3, 0.2, 0.1, 0.15, 0.1, 0.05, 0.05, 0.05]
            },
            'outdoor': {
                'classes': ['person', 'bicycle', 'car', 'bird', 'dog', 'cat', 'bench', 'backpack'],
                'weights': [0.25, 0.15, 0.2, 0.1, 0.1, 0.05, 0.05, 0.1]
            },
            'mixed': {
                'classes': ['person', 'car', 'truck', 'bicycle', 'chair', 'laptop', 'tv', 'cup', 'bird', 'dog'],
                'weights': [0.2, 0.15, 0.1, 0.1, 0.1, 0.1, 0.05, 0.05, 0.08, 0.07]
            }
        }
        
        # Publisher
        self.detection_pub = self.create_publisher(
            String,
            '/yolo_detections',
            10
        )
        
        # Timer for publishing detections
        timer_period = 1.0 / self.detection_frequency
        self.timer = self.create_timer(timer_period, self.publish_simulation_data)
        
        # Simulation state
        self.frame_count = 0
        self.persistent_objects = []  # Objects that persist across frames with movement
        self.start_time = time.time()
        
        self.get_logger().info("YOLO Detection Simulator initialized!")
        self.get_logger().info(f"Publishing to: /yolo_detections")
        self.get_logger().info(f"Scenario: {self.scenario}")
        self.get_logger().info(f"Detection frequency: {self.detection_frequency} Hz")
        self.get_logger().info(f"Detection probability: {self.detection_probability}")
        self.get_logger().info(f"Max objects per frame: {self.max_objects}")
        self.get_logger().info(f"Movement enabled: {self.enable_movement}")
    
    def generate_realistic_bbox(self, class_name: str) -> List[float]:
        """Generate realistic bounding box based on object class"""
        # Typical size ranges for different object classes (normalized coordinates)
        size_ranges = {
            'person': (0.05, 0.3),      # People can vary greatly in size
            'car': (0.1, 0.4),          # Cars are typically medium-large
            'truck': (0.15, 0.5),       # Trucks are larger
            'bus': (0.2, 0.6),          # Buses are large
            'bicycle': (0.03, 0.15),    # Bicycles are smaller
            'motorcycle': (0.04, 0.2),  # Motorcycles are small-medium
            'chair': (0.02, 0.1),       # Furniture varies
            'laptop': (0.02, 0.08),     # Small objects
            'tv': (0.1, 0.3),           # Medium objects
            'bird': (0.01, 0.05),       # Small animals
            'dog': (0.02, 0.15),        # Animals vary
            'cat': (0.01, 0.08),        # Smaller animals
        }
        
        # Get size range for this class, default to medium if unknown
        min_size, max_size = size_ranges.get(class_name, (0.05, 0.2))
        
        # Generate random size within range
        width = random.uniform(min_size, max_size)
        height = random.uniform(min_size, max_size)
        
        # Generate random center position (avoid edges)
        center_x = random.uniform(width/2 + 0.05, 1.0 - width/2 - 0.05)
        center_y = random.uniform(height/2 + 0.05, 1.0 - height/2 - 0.05)
        
        # Convert to corner coordinates
        x1 = max(0.0, center_x - width/2)
        y1 = max(0.0, center_y - height/2)
        x2 = min(1.0, center_x + width/2)
        y2 = min(1.0, center_y + height/2)
        
        return [x1, y1, x2, y2]
    
    def update_object_position(self, bbox: List[float], class_name: str) -> List[float]:
        """Simulate object movement between frames"""
        if not self.enable_movement:
            return bbox
        
        x1, y1, x2, y2 = bbox
        width = x2 - x1
        height = y2 - y1
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        
        # Movement characteristics by class
        movement_speeds = {
            'person': 0.02,      # People walk
            'car': 0.05,         # Cars move faster
            'truck': 0.03,       # Trucks move steadily
            'bus': 0.025,        # Buses move steadily
            'bicycle': 0.04,     # Bicycles move medium speed
            'motorcycle': 0.06,  # Motorcycles move fast
            'bird': 0.08,        # Birds fly quickly
            'dog': 0.03,         # Dogs run around
            'cat': 0.02,         # Cats move slowly
        }
        
        # Static objects don't move much
        static_objects = ['chair', 'dining table', 'tv', 'laptop', 'bench', 'traffic light', 'stop sign']
        if class_name in static_objects:
            speed = 0.005  # Very small random movement
        else:
            speed = movement_speeds.get(class_name, 0.02)
        
        # Add random movement
        dx = random.uniform(-speed, speed)
        dy = random.uniform(-speed, speed)
        
        new_center_x = center_x + dx
        new_center_y = center_y + dy
        
        # Keep object within bounds
        new_center_x = max(width/2, min(1.0 - width/2, new_center_x))
        new_center_y = max(height/2, min(1.0 - height/2, new_center_y))
        
        # Convert back to corner coordinates
        new_x1 = new_center_x - width/2
        new_y1 = new_center_y - height/2
        new_x2 = new_center_x + width/2
        new_y2 = new_center_y + height/2
        
        return [new_x1, new_y1, new_x2, new_y2]
    
    def generate_detections(self) -> List[Dict]:
        """Generate realistic detection data for current frame"""
        detections = []
        
        # Check if we should detect anything this frame
        if random.random() > self.detection_probability:
            return detections
        
        # Get scenario data
        scenario_data = self.scenarios[self.scenario]
        classes = scenario_data['classes']
        weights = scenario_data['weights']
        
        # Update persistent objects with movement
        updated_objects = []
        for obj in self.persistent_objects:
            # Small chance object disappears
            if random.random() < 0.1:  # 10% chance to disappear
                continue
                
            # Update position
            new_bbox = self.update_object_position(obj['bbox'], obj['class_name'])
            # Slightly vary confidence
            new_confidence = obj['confidence'] + random.uniform(-0.05, 0.05)
            new_confidence = max(self.confidence_range[0], min(self.confidence_range[1], new_confidence))
            
            updated_obj = {
                'class_name': obj['class_name'],
                'confidence': new_confidence,
                'bbox': new_bbox
            }
            updated_objects.append(updated_obj)
            detections.append(updated_obj)
        
        self.persistent_objects = updated_objects
        
        # Add new objects
        num_current = len(detections)
        max_new = self.max_objects - num_current
        
        if max_new > 0:
            num_new = random.randint(0, max_new)
            for _ in range(num_new):
                # Choose random class based on scenario weights
                class_name = random.choices(classes, weights=weights)[0]
                confidence = random.uniform(self.confidence_range[0], self.confidence_range[1])
                bbox = self.generate_realistic_bbox(class_name)
                
                detection = {
                    'class_name': class_name,
                    'confidence': confidence,
                    'bbox': bbox
                }
                
                detections.append(detection)
                
                # Some objects persist to next frame
                if random.random() < 0.6:  # 60% chance to persist
                    self.persistent_objects.append(detection.copy())
        
        return detections
    
    def publish_simulation_data(self):
        """Publish simulated detection data"""
        self.frame_count += 1
        current_time = time.time()
        
        # Generate detections for this frame
        detections = self.generate_detections()
        
        # Create realistic header
        header_data = {
            'stamp': {
                'sec': int(current_time),
                'nanosec': int((current_time % 1) * 1e9)
            },
            'frame_id': 'camera_frame'
        }
        
        # Create detection message (same format as real node)
        detection_data = {
            'header': header_data,
            'detections': detections,
            'timestamp': current_time,
            'engine': 'simulator'
        }
        
        # Publish
        msg = String()
        msg.data = json.dumps(detection_data)
        self.detection_pub.publish(msg)
        
        # Log detection info
        if detections:
            detection_info = [f"{d['class_name']}({d['confidence']:.2f})" for d in detections]
            self.get_logger().info(f"Sim Frame {self.frame_count}: {', '.join(detection_info)}")
        else:
            self.get_logger().debug(f"Sim Frame {self.frame_count}: No objects detected")
    
    def get_statistics(self):
        """Get simulator statistics"""
        runtime = time.time() - self.start_time
        return {
            'frames_published': self.frame_count,
            'runtime_seconds': runtime,
            'average_fps': self.frame_count / runtime if runtime > 0 else 0,
            'scenario': self.scenario,
            'persistent_objects': len(self.persistent_objects)
        }

def main(args=None):
    rclpy.init(args=args)
    
    try:
        simulator = YoloDetectionSimulator()
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'simulator' in locals():
            # Print final statistics
            stats = simulator.get_statistics()
            print(f"\nSimulator Statistics:")
            print(f"Frames published: {stats['frames_published']}")
            print(f"Runtime: {stats['runtime_seconds']:.1f}s")
            print(f"Average FPS: {stats['average_fps']:.1f}")
            print(f"Scenario: {stats['scenario']}")
            print(f"Persistent objects: {stats['persistent_objects']}")
            
            simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
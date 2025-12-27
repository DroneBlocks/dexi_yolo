#!/usr/bin/env python3
"""
YOLO Detection Visualizer

Subscribes to compressed images and YOLO detections,
draws bounding boxes, and publishes annotated image.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from dexi_interfaces.msg import YoloDetectionArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class YoloVisualizer(Node):
    def __init__(self):
        super().__init__('yolo_visualizer')

        # Latest detections
        self.detections = []
        self.bridge = CvBridge()

        # Publishers
        self.image_pub = self.create_publisher(
            CompressedImage, '/yolo_debug_image/compressed', 10)
        self.image_raw_pub = self.create_publisher(
            Image, '/yolo_debug_image', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            CompressedImage, '/cam0/image_raw/compressed',
            self.image_callback, 10)

        self.detection_sub = self.create_subscription(
            YoloDetectionArray, '/yolo_detections',
            self.detection_callback, 10)

        self.get_logger().info('YOLO Visualizer started')
        self.get_logger().info('Publishing to /yolo_debug_image/compressed')

    def detection_callback(self, msg):
        self.detections = msg.detections

    def image_callback(self, msg):
        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image is None:
            return

        h, w = image.shape[:2]

        # Draw bounding boxes
        for det in self.detections:
            # Convert normalized coords to pixel coords
            x1 = int(det.bbox[0] * w)
            y1 = int(det.bbox[1] * h)
            x2 = int(det.bbox[2] * w)
            y2 = int(det.bbox[3] * h)

            # Calculate center
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # Draw rectangle
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw center dot
            cv2.circle(image, (cx, cy), 6, (0, 255, 0), -1)

            # Draw center coordinates
            coord_label = f'({cx}, {cy})'
            cv2.putText(image, coord_label, (cx + 10, cy),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

            # Draw label
            label = f'{det.class_name}: {det.confidence:.2f}'
            cv2.putText(image, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish raw image
        raw_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        raw_msg.header = msg.header
        self.image_raw_pub.publish(raw_msg)

        # Publish compressed image
        _, encoded = cv2.imencode('.jpg', image)
        out_msg = CompressedImage()
        out_msg.header = msg.header
        out_msg.format = 'jpeg'
        out_msg.data = encoded.tobytes()
        self.image_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

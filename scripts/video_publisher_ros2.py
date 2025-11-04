#!/usr/bin/env python3
"""
ROS2 Video Publisher for YOLO Testing
Publishes video frames to /cam0/image_raw/compressed for testing YOLO ONNX node
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
import cv2
import numpy as np
import sys
import time
from pathlib import Path


class VideoPublisher(Node):
    """Publishes video frames to ROS2 topic for testing YOLO node"""

    def __init__(self, video_path: str, fps: float = None, loop: bool = True):
        super().__init__('video_publisher')

        self.video_path = video_path
        self.loop = loop

        # Validate video file
        if not Path(video_path).exists():
            self.get_logger().error(f"Video file not found: {video_path}")
            raise FileNotFoundError(f"Video file not found: {video_path}")

        # Open video
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video: {video_path}")
            raise RuntimeError(f"Could not open video: {video_path}")

        # Get video properties
        self.video_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Use specified FPS or video FPS
        self.publish_fps = fps if fps is not None else self.video_fps

        self.get_logger().info("=" * 70)
        self.get_logger().info("VIDEO PUBLISHER")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Video: {video_path}")
        self.get_logger().info(f"Resolution: {self.width}x{self.height}")
        self.get_logger().info(f"Video FPS: {self.video_fps:.2f}")
        self.get_logger().info(f"Publishing FPS: {self.publish_fps:.2f}")
        self.get_logger().info(f"Total frames: {self.frame_count}")
        self.get_logger().info(f"Duration: {self.frame_count/self.video_fps:.2f}s")
        self.get_logger().info(f"Loop mode: {'ON' if self.loop else 'OFF'}")
        self.get_logger().info("=" * 70)

        # Create publisher
        self.image_pub = self.create_publisher(
            CompressedImage,
            '/cam0/image_raw/compressed',
            10
        )

        # Create timer to publish at specified FPS
        timer_period = 1.0 / self.publish_fps
        self.timer = self.create_timer(timer_period, self.publish_frame)

        # Statistics
        self.frame_idx = 0
        self.published_count = 0
        self.start_time = time.time()

        self.get_logger().info("Video publisher started!")
        self.get_logger().info(f"Publishing to: /cam0/image_raw/compressed")
        self.get_logger().info("Press Ctrl+C to stop")

    def publish_frame(self):
        """Read and publish next frame"""
        try:
            ret, frame = self.cap.read()

            # Handle end of video
            if not ret:
                if self.loop:
                    # Restart video
                    self.get_logger().info("End of video reached, looping...")
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    self.frame_idx = 0
                    ret, frame = self.cap.read()

                    if not ret:
                        self.get_logger().error("Failed to restart video")
                        return
                else:
                    self.get_logger().info("End of video reached, stopping...")
                    self.destroy_node()
                    rclpy.shutdown()
                    return

            # Encode frame as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            result, encoded_image = cv2.imencode('.jpg', frame, encode_param)

            if not result:
                self.get_logger().warn("Failed to encode frame")
                return

            # Create CompressedImage message
            msg = CompressedImage()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            msg.format = 'jpeg'
            msg.data = encoded_image.tobytes()

            # Publish
            self.image_pub.publish(msg)

            self.published_count += 1
            self.frame_idx += 1

            # Log progress every 30 frames
            if self.frame_idx % 30 == 0:
                elapsed = time.time() - self.start_time
                fps_actual = self.published_count / elapsed if elapsed > 0 else 0
                progress = (self.frame_idx / self.frame_count) * 100

                self.get_logger().info(
                    f"Frame {self.frame_idx}/{self.frame_count} ({progress:.1f}%) - "
                    f"Published: {self.published_count} frames @ {fps_actual:.1f} FPS"
                )

        except Exception as e:
            self.get_logger().error(f"Error publishing frame: {e}")

    def cleanup(self):
        """Clean up resources"""
        if self.cap is not None:
            self.cap.release()

        # Print statistics
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            self.get_logger().info("\n" + "=" * 70)
            self.get_logger().info("PUBLISHER STATISTICS")
            self.get_logger().info("=" * 70)
            self.get_logger().info(f"Total frames published: {self.published_count}")
            self.get_logger().info(f"Elapsed time: {elapsed:.2f}s")
            self.get_logger().info(f"Average FPS: {self.published_count / elapsed:.2f}")
            self.get_logger().info("=" * 70)


def main(args=None):
    print("\n🎥 ROS2 Video Publisher for YOLO Testing")
    print("=" * 70)

    # Parse arguments
    if len(sys.argv) < 2:
        print("\nUsage: python3 video_publisher_ros2.py <video_path> [fps] [loop]")
        print("\nArguments:")
        print("  video_path  : Path to video file (required)")
        print("  fps         : Publishing rate in FPS (optional, default: video FPS)")
        print("  loop        : Loop video (true/false, default: true)")
        print("\nExamples:")
        print("  python3 video_publisher_ros2.py dexi_camera_all_classes.mp4")
        print("  python3 video_publisher_ros2.py dexi_camera_all_classes.mp4 10")
        print("  python3 video_publisher_ros2.py dexi_camera_all_classes.mp4 10 false")
        print("\n")
        sys.exit(1)

    video_path = sys.argv[1]
    fps = float(sys.argv[2]) if len(sys.argv) > 2 else None
    loop = sys.argv[3].lower() != 'false' if len(sys.argv) > 3 else True

    rclpy.init(args=args)
    node = None

    try:
        node = VideoPublisher(video_path, fps, loop)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        print("\n✓ Video publisher stopped")


if __name__ == '__main__':
    main()

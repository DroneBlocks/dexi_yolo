#!/usr/bin/env python3
"""
Camera simulator node for testing YOLO detection locally
Publishes video frames as compressed images to simulate Pi camera
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
import cv2
import numpy as np
import time
from pathlib import Path

class CameraSimulatorNode(Node):
    """ROS2 node that publishes video frames as compressed images"""

    def __init__(self):
        super().__init__('camera_simulator_node')

        # Parameters
        self.declare_parameter('video_path', 'scripts/dexi_test_flight.mp4')
        self.declare_parameter('fps', 30.0)  # Playback FPS
        self.declare_parameter('loop', True)  # Loop video when finished
        self.declare_parameter('start_paused', False)  # Start in paused state
        self.declare_parameter('compression_quality', 90)  # JPEG quality (0-100)

        # Get parameters
        self.video_path = self.get_parameter('video_path').value
        self.target_fps = self.get_parameter('fps').value
        self.loop_video = self.get_parameter('loop').value
        self.is_paused = self.get_parameter('start_paused').value
        self.compression_quality = self.get_parameter('compression_quality').value

        # Resolve video path
        if not Path(self.video_path).is_absolute():
            # Try relative to current directory first
            if Path(self.video_path).exists():
                self.video_path = str(Path(self.video_path).resolve())
            else:
                self.get_logger().error(f"Video file not found: {self.video_path}")
                raise FileNotFoundError(f"Video file not found: {self.video_path}")

        # Open video file
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video: {self.video_path}")
            raise RuntimeError(f"Could not open video: {self.video_path}")

        # Get video info
        self.video_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.duration = self.frame_count / self.video_fps if self.video_fps > 0 else 0

        # Publisher
        self.image_pub = self.create_publisher(
            CompressedImage,
            '/cam0/image_raw/compressed',
            10
        )

        # Timer for publishing frames at target FPS
        timer_period = 1.0 / self.target_fps
        self.timer = self.create_timer(timer_period, self.publish_frame)

        # State
        self.frame_idx = 0
        self.start_time = time.time()
        self.frames_published = 0

        # JPEG compression parameters
        self.jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, self.compression_quality]

        self.get_logger().info("Camera Simulator initialized!")
        self.get_logger().info(f"Video: {self.video_path}")
        self.get_logger().info(f"Resolution: {self.width}x{self.height}")
        self.get_logger().info(f"Video FPS: {self.video_fps:.2f}")
        self.get_logger().info(f"Playback FPS: {self.target_fps:.2f}")
        self.get_logger().info(f"Total frames: {self.frame_count}")
        self.get_logger().info(f"Duration: {self.duration:.1f}s")
        self.get_logger().info(f"Loop enabled: {self.loop_video}")
        self.get_logger().info(f"Publishing to: /cam0/image_raw/compressed")
        self.get_logger().info(f"Compression quality: {self.compression_quality}%")

        if self.is_paused:
            self.get_logger().info("Started in PAUSED state")

    def publish_frame(self):
        """Read and publish next video frame"""
        if self.is_paused:
            return

        # Read frame
        ret, frame = self.cap.read()

        if not ret:
            if self.loop_video:
                # Restart video from beginning
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.frame_idx = 0
                ret, frame = self.cap.read()
                self.get_logger().info("Video looped to beginning")

                if not ret:
                    self.get_logger().error("Failed to read frame after loop reset")
                    return
            else:
                self.get_logger().info("Video playback complete")
                self.is_paused = True
                return

        # Compress frame to JPEG
        success, encoded_image = cv2.imencode('.jpg', frame, self.jpeg_params)

        if not success:
            self.get_logger().error("Failed to encode frame as JPEG")
            return

        # Create CompressedImage message
        msg = CompressedImage()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.format = 'jpeg'
        msg.data = encoded_image.tobytes()

        # Publish
        self.image_pub.publish(msg)

        self.frame_idx += 1
        self.frames_published += 1

        # Log progress periodically
        if self.frame_idx % 30 == 0:
            progress_pct = (self.frame_idx / self.frame_count) * 100
            runtime = time.time() - self.start_time
            actual_fps = self.frames_published / runtime if runtime > 0 else 0

            self.get_logger().info(
                f"Frame {self.frame_idx}/{self.frame_count} ({progress_pct:.1f}%) | "
                f"Published FPS: {actual_fps:.1f}"
            )

    def pause(self):
        """Pause video playback"""
        self.is_paused = True
        self.get_logger().info("Video playback PAUSED")

    def resume(self):
        """Resume video playback"""
        self.is_paused = False
        self.get_logger().info("Video playback RESUMED")

    def restart(self):
        """Restart video from beginning"""
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        self.frame_idx = 0
        self.get_logger().info("Video restarted from beginning")

    def get_statistics(self):
        """Get simulator statistics"""
        runtime = time.time() - self.start_time
        return {
            'frames_published': self.frames_published,
            'current_frame': self.frame_idx,
            'total_frames': self.frame_count,
            'runtime_seconds': runtime,
            'average_fps': self.frames_published / runtime if runtime > 0 else 0,
            'video_path': self.video_path,
            'resolution': f"{self.width}x{self.height}"
        }

    def cleanup(self):
        """Release video capture resources"""
        if self.cap:
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    simulator = None
    try:
        simulator = CameraSimulatorNode()

        # Inform user of controls
        print("\n" + "="*60)
        print("Camera Simulator Running!")
        print("="*60)
        print("This node publishes video frames to: /cam0/image_raw/compressed")
        print("\nPress Ctrl+C to stop")
        print("="*60 + "\n")

        rclpy.spin(simulator)

    except KeyboardInterrupt:
        print("\n\nStopping camera simulator...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if simulator:
            # Print final statistics
            stats = simulator.get_statistics()
            print(f"\n{'='*60}")
            print("Camera Simulator Statistics:")
            print(f"{'='*60}")
            print(f"Video: {stats['video_path']}")
            print(f"Resolution: {stats['resolution']}")
            print(f"Frames published: {stats['frames_published']}/{stats['total_frames']}")
            print(f"Runtime: {stats['runtime_seconds']:.1f}s")
            print(f"Average FPS: {stats['average_fps']:.1f}")
            print(f"{'='*60}\n")

            simulator.cleanup()
            simulator.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()

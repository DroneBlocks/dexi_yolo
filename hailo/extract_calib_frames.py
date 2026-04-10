#!/usr/bin/env python3
"""
Extract calibration frames from a video for Hailo model compilation.
Samples evenly spaced frames, resizes to the model's input size,
and saves as a .npy array of shape (count, h, w, c) normalized to [0, 1].

Usage:
    python extract_calib_frames.py --video ../scripts/dexi_camera_all_classes.mp4 --output ./calib_set.npy --size 320 --count 64
"""

import argparse
import cv2
import numpy as np


def extract_frames(video_path: str, output_path: str, input_size: int, count: int):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {video_path}")

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if total_frames == 0:
        raise RuntimeError("Video has 0 frames")

    # Sample evenly spaced frame indices
    indices = np.linspace(0, total_frames - 1, count, dtype=int)
    frames = []

    for idx in indices:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(idx))
        ret, frame = cap.read()
        if not ret:
            continue

        resized = cv2.resize(frame, (input_size, input_size))
        # Convert BGR -> RGB and normalize to [0, 1] float32
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        frames.append(rgb.astype(np.float32) / 255.0)

    cap.release()

    calib_set = np.array(frames)
    np.save(output_path, calib_set)
    print(f"Saved calibration set: {output_path} shape={calib_set.shape}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract calibration frames from video")
    parser.add_argument("--video", required=True, help="Path to video file")
    parser.add_argument("--output", default="./calib_set.npy", help="Output .npy file path")
    parser.add_argument("--size", type=int, default=320, help="Model input size (default: 320)")
    parser.add_argument("--count", type=int, default=64, help="Number of frames to extract (default: 64)")
    args = parser.parse_args()

    extract_frames(args.video, args.output, args.size, args.count)

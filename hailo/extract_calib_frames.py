#!/usr/bin/env python3
"""
Extract calibration frames from a video for Hailo model compilation.
Samples evenly spaced frames and resizes to the model's input size.

Usage:
    python extract_calib_frames.py --video ../scripts/dexi_camera_all_classes.mp4 --output ./calib_images --size 320 --count 64
"""

import argparse
import cv2
import numpy as np
from pathlib import Path


def extract_frames(video_path: str, output_dir: str, input_size: int, count: int):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {video_path}")

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if total_frames == 0:
        raise RuntimeError("Video has 0 frames")

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    # Sample evenly spaced frame indices
    indices = np.linspace(0, total_frames - 1, count, dtype=int)
    saved = 0

    for idx in indices:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(idx))
        ret, frame = cap.read()
        if not ret:
            continue

        resized = cv2.resize(frame, (input_size, input_size))
        out_path = out / f"calib_{saved:04d}.jpg"
        cv2.imwrite(str(out_path), resized)
        saved += 1

    cap.release()
    print(f"Extracted {saved} calibration frames to {out}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract calibration frames from video")
    parser.add_argument("--video", required=True, help="Path to video file")
    parser.add_argument("--output", default="./calib_images", help="Output directory")
    parser.add_argument("--size", type=int, default=320, help="Model input size (default: 320)")
    parser.add_argument("--count", type=int, default=64, help="Number of frames to extract (default: 64)")
    args = parser.parse_args()

    extract_frames(args.video, args.output, args.size, args.count)

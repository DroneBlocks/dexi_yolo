#!/usr/bin/env python3
"""
Build a calibration dataset for Hailo INT8 quantization.

Downloads class-balanced images from COCO val2017 for the 6 target classes
(car, motorcycle, truck, bird, cat, dog) and combines them with frames
from deployment videos to create a representative calibration set.

Usage:
    python extract_calib_frames.py --output ./calib_set.npy --size 320 --count 128

    # Video-only mode (no COCO download):
    python extract_calib_frames.py --video ../scripts/dexi_camera_all_classes.mp4 --output ./calib_set.npy
"""

import argparse
import json
import urllib.request
import zipfile
import cv2
import numpy as np
from pathlib import Path

COCO_CLASS_IDS = {
    "car": 3, "motorcycle": 4, "truck": 8,
    "bird": 16, "cat": 17, "dog": 18,
}


def download_coco_annotations(dest_dir):
    url = "http://images.cocodataset.org/annotations/annotations_trainval2017.zip"
    zip_path = dest_dir / "annotations.zip"
    ann_path = dest_dir / "annotations" / "instances_val2017.json"

    if ann_path.exists():
        return ann_path

    print("  Downloading COCO annotations...")
    urllib.request.urlretrieve(url, zip_path)
    with zipfile.ZipFile(zip_path, 'r') as z:
        z.extract("annotations/instances_val2017.json", dest_dir)
    zip_path.unlink()
    return ann_path


def get_coco_images(ann_path, per_class=25):
    with open(ann_path) as f:
        coco = json.load(f)

    cat_images = {cid: set() for cid in COCO_CLASS_IDS.values()}
    for ann in coco["annotations"]:
        cid = ann["category_id"]
        if cid in cat_images:
            cat_images[cid].add(ann["image_id"])

    selected = set()
    np.random.seed(42)
    for name, cid in COCO_CLASS_IDS.items():
        imgs = list(cat_images[cid])
        chosen = np.random.choice(imgs, min(per_class, len(imgs)), replace=False)
        selected.update(chosen)
        print(f"  {name}: {len(chosen)} images")

    id_to_file = {img["id"]: img["file_name"] for img in coco["images"]}
    return [(iid, id_to_file[iid]) for iid in selected]


def download_and_process_coco(image_list, dest_dir, input_size, max_count):
    dest_dir.mkdir(parents=True, exist_ok=True)
    if len(image_list) > max_count:
        np.random.seed(42)
        idx = np.random.choice(len(image_list), max_count, replace=False)
        image_list = [image_list[i] for i in idx]

    frames = []
    for i, (_, filename) in enumerate(image_list):
        path = dest_dir / filename
        if not path.exists():
            try:
                urllib.request.urlretrieve(
                    f"http://images.cocodataset.org/val2017/{filename}", path)
            except Exception:
                continue
        img = cv2.imread(str(path))
        if img is None:
            continue
        resized = cv2.resize(img, (input_size, input_size))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        frames.append(rgb.astype(np.float32) / 255.0)
        if (i + 1) % 20 == 0:
            print(f"  Processed {i+1}/{len(image_list)}...")
    return frames


def extract_video_frames(video_path, input_size, count=32):
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        return []
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    indices = np.linspace(0, total - 1, count, dtype=int)
    frames = []
    for idx in indices:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(idx))
        ret, frame = cap.read()
        if not ret:
            continue
        resized = cv2.resize(frame, (input_size, input_size))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        frames.append(rgb.astype(np.float32) / 255.0)
    cap.release()
    return frames


def main():
    parser = argparse.ArgumentParser(
        description="Build calibration dataset for Hailo quantization")
    parser.add_argument("--video", help="Path to video file (video-only mode)")
    parser.add_argument("--output", default="./calib_set.npy",
                        help="Output .npy path (default: ./calib_set.npy)")
    parser.add_argument("--size", type=int, default=320,
                        help="Model input size (default: 320)")
    parser.add_argument("--count", type=int, default=128,
                        help="Number of calibration frames (default: 128)")
    args = parser.parse_args()

    script_dir = Path(__file__).parent
    frames = []

    if args.video:
        # Video-only mode
        print(f"Extracting {args.count} frames from {args.video}...")
        frames = extract_video_frames(args.video, args.size, args.count)
    else:
        # COCO + video mode
        print("Building class-balanced calibration set from COCO val2017...")
        coco_dir = script_dir / "coco_calib"
        coco_dir.mkdir(exist_ok=True)

        ann_path = download_coco_annotations(coco_dir)
        image_list = get_coco_images(ann_path)
        frames = download_and_process_coco(
            image_list, coco_dir / "images", args.size, args.count)
        print(f"  Got {len(frames)} COCO frames")

        # Add deployment video frames
        video_dir = script_dir.parent / "scripts"
        for video in sorted(video_dir.glob("*.mp4")):
            vframes = extract_video_frames(video, args.size, 32)
            frames.extend(vframes)
            print(f"  Added {len(vframes)} frames from {video.name}")

    # Trim or pad to target count
    if len(frames) > args.count:
        np.random.seed(42)
        idx = np.random.choice(len(frames), args.count, replace=False)
        frames = [frames[i] for i in idx]

    calib_set = np.array(frames)
    np.save(args.output, calib_set)
    print(f"Saved calibration set: {args.output} shape={calib_set.shape}")


if __name__ == "__main__":
    main()

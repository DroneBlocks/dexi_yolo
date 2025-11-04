#!/usr/bin/env python3
"""
Video-based ONNX model inference debugger
Processes MP4 video from drone footage to test model performance
"""

import cv2
import numpy as np
import onnxruntime as ort
import sys
import time
from pathlib import Path
from typing import List, Tuple

class Detection:
    """Represents a single detection"""
    def __init__(self, class_name: str, confidence: float, bbox: List[float]):
        self.class_name = class_name
        self.confidence = confidence
        self.bbox = bbox  # [x1, y1, x2, y2] normalized

def preprocess_frame_ros_style(frame: np.ndarray, input_size: int = 320) -> np.ndarray:
    """
    Method 1: ROS node style - simple resize
    """
    # Resize
    resized = cv2.resize(frame, (input_size, input_size))

    # Convert BGR to RGB
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

    # Normalize
    normalized = rgb.astype(np.float32) / 255.0

    # Transpose HWC to CHW
    chw = normalized.transpose(2, 0, 1)

    # Add batch dimension
    batch = np.expand_dims(chw, axis=0)

    return batch

def preprocess_frame_letterbox(frame: np.ndarray, input_size: int = 320) -> Tuple[np.ndarray, dict]:
    """
    Method 2: Ultralytics style - letterbox with aspect ratio preservation
    Returns preprocessed tensor and metadata for reverse transformation
    """
    orig_h, orig_w = frame.shape[:2]

    # Calculate scale
    scale = min(input_size / orig_w, input_size / orig_h)
    new_w = int(orig_w * scale)
    new_h = int(orig_h * scale)

    # Resize with aspect ratio preserved
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    # Create padded image (letterbox)
    padded = np.full((input_size, input_size, 3), 114, dtype=np.uint8)

    # Calculate padding offsets (center the image)
    top = (input_size - new_h) // 2
    left = (input_size - new_w) // 2

    padded[top:top+new_h, left:left+new_w] = resized

    # Convert BGR to RGB
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)

    # Normalize
    normalized = rgb.astype(np.float32) / 255.0

    # Transpose and batch
    chw = normalized.transpose(2, 0, 1)
    batch = np.expand_dims(chw, axis=0)

    # Metadata for coordinate transformation
    metadata = {
        'scale': scale,
        'pad_top': top,
        'pad_left': left,
        'new_w': new_w,
        'new_h': new_h,
        'orig_w': orig_w,
        'orig_h': orig_h
    }

    return batch, metadata

def postprocess_detections(output: np.ndarray, original_shape: Tuple[int, int],
                          class_names: List[str], conf_threshold: float = 0.5,
                          input_size: int = 320) -> List[Detection]:
    """
    Process YOLO output to extract detections
    """
    detections = []

    # YOLOv8 output shape: (1, num_classes+4, num_predictions)
    output_t = output[0].T  # Transpose to (num_predictions, channels)

    # Extract bbox and class scores
    bbox_coords = output_t[:, :4]
    class_scores = output_t[:, 4:]

    # Apply sigmoid if scores are raw logits
    if class_scores.min() < 0 or class_scores.max() > 1.5:
        class_scores = 1 / (1 + np.exp(-class_scores))

    # Get max class scores
    max_scores = np.max(class_scores, axis=1)
    max_classes = np.argmax(class_scores, axis=1)

    # Filter by confidence
    valid_indices = max_scores >= conf_threshold

    orig_h, orig_w = original_shape
    scale_x = orig_w / input_size
    scale_y = orig_h / input_size

    for idx in np.where(valid_indices)[0]:
        # Get bbox in center format (cx, cy, w, h)
        cx, cy, w, h = bbox_coords[idx]

        # Convert to corner format and scale to original image
        x1 = max(0, (cx - w / 2) * scale_x / orig_w)
        y1 = max(0, (cy - h / 2) * scale_y / orig_h)
        x2 = min(1, (cx + w / 2) * scale_x / orig_w)
        y2 = min(1, (cy + h / 2) * scale_y / orig_h)

        confidence = max_scores[idx]
        class_idx = max_classes[idx]
        class_name = class_names[class_idx] if class_idx < len(class_names) else f"class_{class_idx}"

        detections.append(Detection(class_name, float(confidence), [x1, y1, x2, y2]))

    return detections

def draw_detections(frame: np.ndarray, detections: List[Detection]) -> np.ndarray:
    """Draw bounding boxes and labels on frame"""
    annotated = frame.copy()
    h, w = frame.shape[:2]

    for det in detections:
        x1, y1, x2, y2 = det.bbox

        # Convert normalized coords to pixel coords
        x1_px = int(x1 * w)
        y1_px = int(y1 * h)
        x2_px = int(x2 * w)
        y2_px = int(y2 * h)

        # Draw box
        color = (0, 255, 0)  # Green
        cv2.rectangle(annotated, (x1_px, y1_px), (x2_px, y2_px), color, 2)

        # Draw label
        label = f"{det.class_name}: {det.confidence:.2f}"
        (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        cv2.rectangle(annotated, (x1_px, y1_px - label_h - 10), (x1_px + label_w, y1_px), color, -1)
        cv2.putText(annotated, label, (x1_px, y1_px - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

    return annotated

def process_video(video_path: str, model_path: str, class_names: List[str],
                 input_size: int = 320, conf_threshold: float = 0.5,
                 use_letterbox: bool = True, save_output: bool = True,
                 max_frames: int = None):
    """Process video file and test ONNX inference"""

    print("=" * 70)
    print("🎥 VIDEO PROCESSING")
    print("=" * 70)
    print(f"Video: {video_path}")
    print(f"Model: {model_path}")
    print(f"Input size: {input_size}x{input_size}")
    print(f"Confidence threshold: {conf_threshold}")
    print(f"Preprocessing: {'Letterbox' if use_letterbox else 'Simple Resize (ROS Style)'}")
    print(f"Classes: {class_names}")

    # Load video
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"❌ Could not open video: {video_path}")
        return

    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"\n📊 Video Info:")
    print(f"  Resolution: {width}x{height}")
    print(f"  FPS: {fps:.2f}")
    print(f"  Total frames: {frame_count}")
    print(f"  Duration: {frame_count/fps:.2f}s")

    if max_frames:
        print(f"  Processing limit: {max_frames} frames")

    # Load ONNX model
    print(f"\n⏳ Loading ONNX model...")
    sess_options = ort.SessionOptions()
    sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    session = ort.InferenceSession(model_path, sess_options=sess_options)

    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    print(f"✓ Model loaded: {input_name} -> {output_name}")

    # Setup output video if saving
    output_writer = None
    if save_output:
        output_path = str(Path(video_path).stem) + f"_output_{'letterbox' if use_letterbox else 'resize'}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        output_writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        print(f"💾 Saving output to: {output_path}")

    # Statistics
    stats = {
        'frames_processed': 0,
        'frames_with_detections': 0,
        'total_detections': 0,
        'class_counts': {name: 0 for name in class_names},
        'total_inference_time': 0,
        'total_preprocess_time': 0,
        'total_postprocess_time': 0
    }

    print(f"\n🚀 Processing frames...")
    print("=" * 70)

    frame_idx = 0
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            if max_frames and frame_idx >= max_frames:
                break

            original_shape = frame.shape[:2]

            # Preprocess
            t_start = time.time()
            if use_letterbox:
                input_tensor, metadata = preprocess_frame_letterbox(frame, input_size)
            else:
                input_tensor = preprocess_frame_ros_style(frame, input_size)
            t_preprocess = time.time() - t_start

            # Inference
            t_start = time.time()
            outputs = session.run([output_name], {input_name: input_tensor})
            t_inference = time.time() - t_start

            # Postprocess
            t_start = time.time()
            detections = postprocess_detections(outputs[0], original_shape, class_names,
                                               conf_threshold, input_size)
            t_postprocess = time.time() - t_start

            # Update stats
            stats['frames_processed'] += 1
            stats['total_preprocess_time'] += t_preprocess
            stats['total_inference_time'] += t_inference
            stats['total_postprocess_time'] += t_postprocess

            if detections:
                stats['frames_with_detections'] += 1
                stats['total_detections'] += len(detections)

                for det in detections:
                    if det.class_name in stats['class_counts']:
                        stats['class_counts'][det.class_name] += 1

                # Print detection info
                det_str = ", ".join([f"{d.class_name}({d.confidence:.2f})" for d in detections])
                print(f"Frame {frame_idx:4d}: {len(detections)} detections - {det_str}")

            # Draw and save
            if save_output:
                annotated_frame = draw_detections(frame, detections)

                # Add stats overlay
                info_text = f"Frame: {frame_idx} | Detections: {len(detections)} | Inference: {t_inference*1000:.1f}ms"
                cv2.putText(annotated_frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                           0.7, (255, 255, 255), 2)

                output_writer.write(annotated_frame)

            frame_idx += 1

            # Progress indicator
            if frame_idx % 30 == 0:
                progress = (frame_idx / (max_frames or frame_count)) * 100
                print(f"  Progress: {progress:.1f}% ({frame_idx}/{max_frames or frame_count} frames)")

    except KeyboardInterrupt:
        print("\n⚠️  Processing interrupted by user")

    finally:
        cap.release()
        if output_writer:
            output_writer.release()

    # Print summary
    print("\n" + "=" * 70)
    print("📊 PROCESSING SUMMARY")
    print("=" * 70)

    if stats['frames_processed'] > 0:
        print(f"\n✓ Frames processed: {stats['frames_processed']}")
        print(f"✓ Frames with detections: {stats['frames_with_detections']} "
              f"({stats['frames_with_detections']/stats['frames_processed']*100:.1f}%)")
        print(f"✓ Total detections: {stats['total_detections']}")
        print(f"✓ Avg detections per frame: {stats['total_detections']/stats['frames_processed']:.2f}")

        print(f"\n⏱️  Timing (avg per frame):")
        print(f"  Preprocessing: {stats['total_preprocess_time']/stats['frames_processed']*1000:.2f}ms")
        print(f"  Inference: {stats['total_inference_time']/stats['frames_processed']*1000:.2f}ms")
        print(f"  Postprocessing: {stats['total_postprocess_time']/stats['frames_processed']*1000:.2f}ms")
        print(f"  Total: {(stats['total_preprocess_time'] + stats['total_inference_time'] + stats['total_postprocess_time'])/stats['frames_processed']*1000:.2f}ms")

        print(f"\n🏷️  Class distribution:")
        for class_name, count in stats['class_counts'].items():
            if count > 0:
                print(f"  {class_name}: {count}")

        if save_output:
            print(f"\n💾 Output video saved!")
    else:
        print("❌ No frames processed")

def main():
    print("🎥 ONNX Video Inference Debugger")
    print("=" * 70)

    # Configuration - MODIFY THESE
    video_path = "dexi_camera_all_classes.mp4"  # Your drone footage
    model_path = "models/best.onnx"
    class_names = ['car', 'motorcycle', 'truck', 'bird', 'cat', 'dog']

    input_size = 320
    conf_threshold = 0.5  # Lower if needed (try 0.25)
    use_letterbox = False  # Set to True to test letterbox preprocessing
    max_frames = 3000  # Set to number to limit processing (e.g., 100 for testing)

    # Check files
    if not Path(video_path).exists():
        print(f"❌ Video not found: {video_path}")
        print("Please provide path to your MP4 file")
        return

    if not Path(model_path).exists():
        print(f"❌ Model not found: {model_path}")
        print("Please check model path")
        return

    print("\n🔧 Configuration:")
    print(f"  Video: {video_path}")
    print(f"  Model: {model_path}")
    print(f"  Input size: {input_size}x{input_size}")
    print(f"  Confidence threshold: {conf_threshold}")
    print(f"  Use letterbox: {use_letterbox}")
    print(f"  Max frames: {max_frames or 'All'}")

    print("\n" + "=" * 70)
    print("Starting processing...")
    print("Press Ctrl+C to stop early")
    print("=" * 70 + "\n")

    # Process video
    process_video(
        video_path=video_path,
        model_path=model_path,
        class_names=class_names,
        input_size=input_size,
        conf_threshold=conf_threshold,
        use_letterbox=use_letterbox,
        save_output=True,
        max_frames=max_frames
    )

    print("\n✓ Done! Check the output video to see detections.")
    print("\n💡 Tips:")
    print("  - If no detections: try lowering conf_threshold or changing input_size")
    print("  - If detections are wrong: try use_letterbox=True")
    print("  - Compare with: yolo predict source=test_video.mp4 model=models/best_optimized.onnx")

if __name__ == "__main__":
    main()

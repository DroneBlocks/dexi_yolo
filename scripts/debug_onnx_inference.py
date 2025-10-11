#!/usr/bin/env python3
"""
Diagnostic script to debug ONNX model inference
This script helps identify differences between yolo predict and ROS node inference
"""

import cv2
import numpy as np
import onnxruntime as ort
import sys
from pathlib import Path

def inspect_model(model_path):
    """Inspect ONNX model inputs and outputs"""
    print("=" * 60)
    print("MODEL INSPECTION")
    print("=" * 60)

    session = ort.InferenceSession(model_path)

    print("\n📥 INPUTS:")
    for inp in session.get_inputs():
        print(f"  Name: {inp.name}")
        print(f"  Shape: {inp.shape}")
        print(f"  Type: {inp.type}")

    print("\n📤 OUTPUTS:")
    for out in session.get_outputs():
        print(f"  Name: {out.name}")
        print(f"  Shape: {out.shape}")
        print(f"  Type: {out.type}")

    return session

def preprocess_image_method1(image_path, input_size=640):
    """Method 1: Similar to current ROS node"""
    print("\n" + "=" * 60)
    print("PREPROCESSING - METHOD 1 (Current ROS Node Style)")
    print("=" * 60)

    # Read image
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError(f"Could not read image: {image_path}")

    print(f"Original image shape: {image.shape}")
    print(f"Original dtype: {image.dtype}")
    print(f"Original range: [{image.min()}, {image.max()}]")

    # Resize
    resized = cv2.resize(image, (input_size, input_size))
    print(f"\nAfter resize: {resized.shape}")

    # Convert BGR to RGB
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    print(f"After BGR->RGB: {rgb.shape}")

    # Normalize and convert to float32
    normalized = rgb.astype(np.float32) / 255.0
    print(f"After normalize: shape={normalized.shape}, dtype={normalized.dtype}")
    print(f"Normalized range: [{normalized.min():.4f}, {normalized.max():.4f}]")

    # Transpose HWC to CHW
    chw = normalized.transpose(2, 0, 1)
    print(f"After HWC->CHW: {chw.shape}")

    # Add batch dimension
    batch = np.expand_dims(chw, axis=0)
    print(f"After add batch: {batch.shape}")

    return batch, image

def preprocess_image_method2(image_path, input_size=640):
    """Method 2: Ultralytics style (letterbox padding)"""
    print("\n" + "=" * 60)
    print("PREPROCESSING - METHOD 2 (Ultralytics Letterbox Style)")
    print("=" * 60)

    # Read image
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError(f"Could not read image: {image_path}")

    print(f"Original image shape: {image.shape}")
    orig_h, orig_w = image.shape[:2]

    # Calculate letterbox padding (maintains aspect ratio)
    scale = min(input_size / orig_w, input_size / orig_h)
    new_w = int(orig_w * scale)
    new_h = int(orig_h * scale)

    print(f"Scale factor: {scale:.4f}")
    print(f"Scaled size: {new_w}x{new_h}")

    # Resize with aspect ratio preserved
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    # Create padded image (letterbox)
    padded = np.full((input_size, input_size, 3), 114, dtype=np.uint8)  # Gray padding

    # Calculate padding offsets (center the image)
    top = (input_size - new_h) // 2
    left = (input_size - new_w) // 2

    padded[top:top+new_h, left:left+new_w] = resized
    print(f"After letterbox padding: {padded.shape}")
    print(f"Padding: top={top}, left={left}")

    # Convert BGR to RGB
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)

    # Normalize
    normalized = rgb.astype(np.float32) / 255.0
    print(f"Normalized range: [{normalized.min():.4f}, {normalized.max():.4f}]")

    # Transpose and batch
    chw = normalized.transpose(2, 0, 1)
    batch = np.expand_dims(chw, axis=0)
    print(f"Final tensor shape: {batch.shape}")

    return batch, image

def run_inference(session, input_tensor):
    """Run inference and return raw outputs"""
    print("\n" + "=" * 60)
    print("RUNNING INFERENCE")
    print("=" * 60)

    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name

    print(f"Input name: {input_name}")
    print(f"Input shape: {input_tensor.shape}")
    print(f"Input dtype: {input_tensor.dtype}")

    # Run inference
    outputs = session.run([output_name], {input_name: input_tensor})
    output = outputs[0]

    print(f"\nOutput name: {output_name}")
    print(f"Output shape: {output.shape}")
    print(f"Output dtype: {output.dtype}")
    print(f"Output range: [{output.min():.4f}, {output.max():.4f}]")

    return output

def analyze_output(output, class_names=None, conf_threshold=0.5):
    """Analyze raw model output to understand its format"""
    print("\n" + "=" * 60)
    print("ANALYZING OUTPUT FORMAT")
    print("=" * 60)

    print(f"Raw output shape: {output.shape}")

    # YOLOv8 typically outputs (1, num_classes+4, num_predictions)
    if len(output.shape) == 3:
        batch_size, channels, num_predictions = output.shape
        num_classes = channels - 4

        print(f"  Batch size: {batch_size}")
        print(f"  Channels: {channels}")
        print(f"  Num predictions: {num_predictions}")
        print(f"  Inferred num_classes: {num_classes}")

        # Transpose to (num_predictions, channels)
        output_t = output[0].T  # (num_predictions, channels)
        print(f"\nTransposed shape: {output_t.shape}")

        # Analyze bbox coordinates (first 4 channels)
        bbox_coords = output_t[:, :4]
        print(f"\n📊 Bounding box coordinates (first 4 channels):")
        print(f"  Shape: {bbox_coords.shape}")
        print(f"  Range: [{bbox_coords.min():.2f}, {bbox_coords.max():.2f}]")
        print(f"  Mean: {bbox_coords.mean():.2f}")

        # Analyze class scores (remaining channels)
        class_scores = output_t[:, 4:]
        print(f"\n📊 Class scores (remaining {num_classes} channels):")
        print(f"  Shape: {class_scores.shape}")
        print(f"  Range: [{class_scores.min():.4f}, {class_scores.max():.4f}]")
        print(f"  Mean: {class_scores.mean():.4f}")

        # Check if scores look like raw logits (need sigmoid) or probabilities
        if class_scores.min() < 0 or class_scores.max() > 1.5:
            print("  ⚠️  Scores appear to be RAW LOGITS (need sigmoid activation)")
            # Apply sigmoid
            class_scores_sigmoid = 1 / (1 + np.exp(-class_scores))
            print(f"  After sigmoid: [{class_scores_sigmoid.min():.4f}, {class_scores_sigmoid.max():.4f}]")
            class_scores = class_scores_sigmoid
        else:
            print("  ✓ Scores appear to be probabilities (sigmoid already applied)")

        # Find max scores across all predictions
        max_scores = np.max(class_scores, axis=1)
        max_classes = np.argmax(class_scores, axis=1)

        print(f"\n📊 Maximum scores statistics:")
        print(f"  Max score overall: {max_scores.max():.4f}")
        print(f"  Min score overall: {max_scores.min():.4f}")
        print(f"  Mean score: {max_scores.mean():.4f}")
        print(f"  Median score: {np.median(max_scores):.4f}")

        # Count how many exceed threshold
        above_threshold = np.sum(max_scores >= conf_threshold)
        print(f"\n🎯 Detections above {conf_threshold} threshold: {above_threshold}/{num_predictions}")

        if above_threshold > 0:
            print(f"\nTop 10 detections:")
            top_indices = np.argsort(max_scores)[-10:][::-1]

            for idx in top_indices:
                conf = max_scores[idx]
                cls = max_classes[idx]
                bbox = bbox_coords[idx]

                class_name = class_names[cls] if class_names and cls < len(class_names) else f"class_{cls}"

                if conf >= conf_threshold:
                    marker = "✓"
                else:
                    marker = "✗"

                print(f"  {marker} {class_name}: {conf:.4f} | bbox: [{bbox[0]:.1f}, {bbox[1]:.1f}, {bbox[2]:.1f}, {bbox[3]:.1f}]")
        else:
            print("\n⚠️  NO DETECTIONS found above threshold!")
            print(f"   Consider lowering threshold or checking preprocessing")

            # Show top scores anyway
            print(f"\nTop 5 scores (below threshold):")
            top_indices = np.argsort(max_scores)[-5:][::-1]
            for idx in top_indices:
                conf = max_scores[idx]
                cls = max_classes[idx]
                class_name = class_names[cls] if class_names and cls < len(class_names) else f"class_{cls}"
                print(f"  {class_name}: {conf:.4f}")

    return output

def main():
    print("🔍 ONNX Model Inference Debugger")
    print("=" * 60)

    # Configuration
    model_path = "models/best.onnx"
    test_image = "test_image.jpg"  # You'll need to provide a test image

    # Your custom class names
    class_names = ['car', 'motorcycle', 'truck', 'bird', 'cat', 'dog']

    print(f"\n📁 Model: {model_path}")
    print(f"📷 Test image: {test_image}")
    print(f"🏷️  Classes: {class_names}")

    # Check if files exist
    if not Path(model_path).exists():
        print(f"\n❌ Model not found: {model_path}")
        return

    if not Path(test_image).exists():
        print(f"\n❌ Test image not found: {test_image}")
        print("Please provide a test image (jpg/png) as 'test_image.jpg'")
        print("Or modify the script to use your image path")
        return

    # Step 1: Inspect model
    session = inspect_model(model_path)

    # Step 2: Try preprocessing method 1 (current ROS node)
    input_tensor1, original_image = preprocess_image_method1(test_image, input_size=320)
    output1 = run_inference(session, input_tensor1)

    print("\n" + "=" * 60)
    print("RESULTS - METHOD 1 (ROS Node Style)")
    print("=" * 60)
    analyze_output(output1, class_names=class_names, conf_threshold=0.5)

    # Step 3: Try preprocessing method 2 (ultralytics letterbox)
    print("\n\n")
    input_tensor2, _ = preprocess_image_method2(test_image, input_size=320)
    output2 = run_inference(session, input_tensor2)

    print("\n" + "=" * 60)
    print("RESULTS - METHOD 2 (Letterbox Style)")
    print("=" * 60)
    analyze_output(output2, class_names=class_names, conf_threshold=0.5)

    print("\n\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print("\n✓ Script completed! Review the output above to see:")
    print("  1. Model input/output shapes")
    print("  2. Whether sigmoid activation is needed")
    print("  3. Which preprocessing method works better")
    print("  4. What detection scores you're getting")
    print("\nUse this information to fix the ROS node preprocessing/postprocessing!")

if __name__ == "__main__":
    main()

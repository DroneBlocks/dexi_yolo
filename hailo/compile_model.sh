#!/bin/bash
# Compile a custom ONNX YOLO model for Hailo 8L
#
# Prerequisites:
#   pip install hailo_dataflow_compiler
#   (Download .whl from https://hailo.ai/developer-zone/ if pip install fails)
#
# Usage:
#   ./compile_model.sh
#
# This script:
#   1. Extracts calibration frames from the training video
#   2. Optimizes/quantizes the ONNX model for Hailo 8L
#   3. Compiles to .hef format
#
# Output: models/best_optimized_h8l.hef

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
YOLO_DIR="$(dirname "$SCRIPT_DIR")"
MODEL="${YOLO_DIR}/models/best_optimized.onnx"
VIDEO="${YOLO_DIR}/scripts/dexi_camera_all_classes.mp4"
CALIB_DIR="${SCRIPT_DIR}/calib_images"
OUTPUT_HAR="${SCRIPT_DIR}/best_optimized_h8l.har"
OUTPUT_HEF="${YOLO_DIR}/models/best_optimized_h8l.hef"
INPUT_SIZE=320
CALIB_COUNT=64
HW_ARCH="hailo8l"

echo "================================================"
echo "  Hailo 8L Model Compilation"
echo "================================================"
echo "  ONNX model:  ${MODEL}"
echo "  Video:       ${VIDEO}"
echo "  Input size:  ${INPUT_SIZE}x${INPUT_SIZE}"
echo "  HW target:   ${HW_ARCH}"
echo "================================================"
echo ""

# Check prerequisites
if ! command -v hailo &> /dev/null; then
    echo "ERROR: Hailo DFC not found."
    echo ""
    echo "Install it:"
    echo "  pip install hailo_dataflow_compiler"
    echo ""
    echo "Or download the .whl from https://hailo.ai/developer-zone/"
    echo "then: pip install hailo_dataflow_compiler-<version>.whl"
    exit 1
fi

if [ ! -f "$MODEL" ]; then
    echo "ERROR: ONNX model not found at ${MODEL}"
    exit 1
fi

# Step 1: Extract calibration frames
echo "[1/3] Extracting calibration frames..."
if [ -f "$VIDEO" ]; then
    python3 "${SCRIPT_DIR}/extract_calib_frames.py" \
        --video "$VIDEO" \
        --output "$CALIB_DIR" \
        --size "$INPUT_SIZE" \
        --count "$CALIB_COUNT"
else
    echo "WARNING: Video not found at ${VIDEO}"
    echo "Please provide calibration images in ${CALIB_DIR}/"
    echo "(At least 32 images, resized to ${INPUT_SIZE}x${INPUT_SIZE})"
    if [ ! -d "$CALIB_DIR" ] || [ -z "$(ls -A "$CALIB_DIR" 2>/dev/null)" ]; then
        echo "ERROR: No calibration images found. Cannot continue."
        exit 1
    fi
fi

# Step 2: Optimize (parse + quantize)
echo ""
echo "[2/3] Optimizing model (parse + quantize)..."
echo "       This may take several minutes..."
hailo optimize "$MODEL" \
    --hw-arch "$HW_ARCH" \
    --calib-path "$CALIB_DIR" \
    --output-har-path "$OUTPUT_HAR"

echo "  HAR saved: ${OUTPUT_HAR}"

# Step 3: Compile to HEF
echo ""
echo "[3/3] Compiling to HEF..."
hailo compile "$OUTPUT_HAR" \
    --hw-arch "$HW_ARCH" \
    --output-file "$OUTPUT_HEF"

echo ""
echo "================================================"
echo "  Compilation complete!"
echo "================================================"
echo "  HEF model: ${OUTPUT_HEF}"
echo ""
echo "  Copy to the Pi:"
echo "    scp ${OUTPUT_HEF} dexi@192.168.68.63:/home/dexi/dexi_ws/src/dexi_yolo/models/"
echo ""
echo "  Launch on the Pi:"
echo "    ros2 launch dexi_yolo yolo_hailo_launch.py hef_path:=/home/dexi/dexi_ws/src/dexi_yolo/models/best_optimized_h8l.hef"
echo "================================================"

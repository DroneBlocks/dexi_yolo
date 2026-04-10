#!/bin/bash
# Compile a custom ONNX YOLO model for Hailo 8L
#
# Prerequisites:
#   pip install hailo_dataflow_compiler
#   pip install opencv-python numpy
#
# Usage:
#   ./compile_model.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
YOLO_DIR="$(dirname "$SCRIPT_DIR")"
MODEL="${YOLO_DIR}/models/best_optimized.onnx"
VIDEO="${YOLO_DIR}/scripts/dexi_camera_all_classes.mp4"
CALIB_NPY="${SCRIPT_DIR}/calib_set.npy"
PARSED_HAR="${SCRIPT_DIR}/best_optimized_parsed.har"
OPTIMIZED_HAR="${SCRIPT_DIR}/best_optimized_h8l.har"
OUTPUT_DIR="${YOLO_DIR}/models"
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
    exit 1
fi

if [ ! -f "$MODEL" ]; then
    echo "ERROR: ONNX model not found at ${MODEL}"
    exit 1
fi

# Step 1: Extract calibration frames to .npy
echo "[1/4] Extracting calibration frames..."
if [ -f "$VIDEO" ]; then
    python3 "${SCRIPT_DIR}/extract_calib_frames.py" \
        --video "$VIDEO" \
        --output "$CALIB_NPY" \
        --size "$INPUT_SIZE" \
        --count "$CALIB_COUNT"
else
    if [ ! -f "$CALIB_NPY" ]; then
        echo "ERROR: No calibration data found. Cannot continue."
        exit 1
    fi
    echo "Using existing ${CALIB_NPY}"
fi

# Step 2: Parse ONNX to HAR
echo ""
echo "[2/4] Parsing ONNX model..."
hailo parser onnx "$MODEL" \
    --hw-arch "$HW_ARCH" \
    --har-path "$PARSED_HAR" \
    -y

echo "  Parsed HAR: ${PARSED_HAR}"

# Step 3: Optimize (quantize)
echo ""
echo "[3/4] Optimizing model (quantize)..."
echo "       This may take several minutes..."
hailo optimize "$PARSED_HAR" \
    --hw-arch "$HW_ARCH" \
    --calib-set-path "$CALIB_NPY" \
    --output-har-path "$OPTIMIZED_HAR"

echo "  Optimized HAR: ${OPTIMIZED_HAR}"

# Step 4: Compile to HEF
echo ""
echo "[4/4] Compiling to HEF..."
hailo compiler "$OPTIMIZED_HAR" \
    --hw-arch "$HW_ARCH" \
    --output-dir "$OUTPUT_DIR"

echo ""
echo "================================================"
echo "  Compilation complete!"
echo "================================================"
echo "  HEF model: ${OUTPUT_DIR}/"
echo ""
echo "  Copy to the Pi:"
echo "    scp ${OUTPUT_DIR}/*.hef dexi@192.168.68.63:/home/dexi/dexi_ws/src/dexi_yolo/models/"
echo "================================================"

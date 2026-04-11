#!/bin/bash
# Compile a custom ONNX YOLO model for Hailo 8L
#
# Prerequisites:
#   pip install hailo_dataflow_compiler
#   pip install opencv-python-headless numpy
#
# Usage:
#   ./compile_model.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
YOLO_DIR="$(dirname "$SCRIPT_DIR")"
MODEL="${YOLO_DIR}/models/best_optimized.onnx"
CALIB_NPY="${SCRIPT_DIR}/calib_set.npy"
PARSED_HAR="${SCRIPT_DIR}/best_optimized_parsed.har"
OPTIMIZED_HAR="${SCRIPT_DIR}/best_optimized_h8l.har"
OUTPUT_DIR="${YOLO_DIR}/models"
INPUT_SIZE=320
CALIB_COUNT=128
HW_ARCH="hailo8l"

echo "================================================"
echo "  Hailo 8L Model Compilation"
echo "================================================"
echo "  ONNX model:  ${MODEL}"
echo "  Input size:  ${INPUT_SIZE}x${INPUT_SIZE}"
echo "  Calib count: ${CALIB_COUNT}"
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

# Step 1: Build calibration dataset (COCO + video frames)
echo "[1/4] Building calibration dataset..."
python3 "${SCRIPT_DIR}/extract_calib_frames.py" \
    --output "$CALIB_NPY" \
    --size "$INPUT_SIZE" \
    --count "$CALIB_COUNT"

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
echo "  HEF model: ${OUTPUT_DIR}/best_optimized.hef"
echo ""
echo "  Copy to the Pi:"
echo "    scp ${OUTPUT_DIR}/best_optimized.hef dexi@<pi-ip>:/home/dexi/dexi_ws/src/dexi_yolo/models/"
echo "================================================"

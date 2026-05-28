# DEXI YOLO ROS2 Package

A ROS2 package for YOLO object detection optimized for Raspberry Pi. Provides PyTorch, ONNX Runtime, and Hailo 8L accelerated implementations with desktop simulation support.

## Features

- **Multiple Detection Engines**: PyTorch, ONNX Runtime, and Hailo 8L NPU
- **Pi CM4 Optimized**: CPU thread limiting, memory optimizations, NMS
- **Desktop Simulation**: Test without hardware using realistic detection data
- **Offline Ready**: Models included for deployment without internet
- **Unified Interface**: All engines publish to `/yolo_detections`

## Quick Start

```bash
# Pi 5 with Hailo M.2 hat (recommended when available)
ros2 launch dexi_yolo yolo_hailo_launch.py

# Pi CM4 / Pi without Hailo — ONNX CPU inference
ros2 launch dexi_yolo yolo_onnx_launch.py

# PyTorch (heavier, mostly for reference)
ros2 launch dexi_yolo yolo_launch.py

# Or run nodes directly
ros2 run dexi_yolo dexi_yolo_node_hailo.py  # Hailo 8L
ros2 run dexi_yolo dexi_yolo_node_onnx.py   # ONNX
ros2 run dexi_yolo dexi_yolo_node.py        # PyTorch

# Desktop simulation
ros2 run dexi_yolo dexi_yolo_simulator
```

## Package Structure

```
dexi_yolo/
├── src/
│   ├── dexi_yolo_node.py           # PyTorch implementation
│   ├── dexi_yolo_node_onnx.py      # ONNX Runtime (optimized)
│   ├── dexi_yolo_node_hailo.py     # Hailo 8L accelerated inference
│   ├── dexi_yolo_simulator.py      # Detection simulator
│   └── camera_simulator_node.py    # Camera/video simulator for testing
├── hailo/
│   ├── compile_model.sh            # ONNX → HEF compilation script
│   └── extract_calib_frames.py     # Calibration frame extractor
├── models/
│   ├── yolov8n.pt                  # Stock YOLOv8n PyTorch (reference)
│   ├── yolov8n.onnx                # Stock YOLOv8n ONNX (reference)
│   ├── best_optimized.pt           # Custom fine-tuned PyTorch model
│   ├── best_optimized.onnx         # Custom fine-tuned ONNX model (320x320)
│   └── best_optimized.hef          # Custom fine-tuned Hailo 8L compiled model
├── launch/
│   ├── yolo_launch.py              # PyTorch node launch file
│   ├── yolo_onnx_launch.py         # ONNX node launch file
│   ├── yolo_hailo_launch.py        # Hailo 8L launch file (recommended for Pi 5)
│   └── camera_simulator_launch.py  # Local testing with camera simulator
├── scripts/                        # Debug and testing scripts
│   ├── debug_onnx_inference.py     # Test single image
│   ├── debug_onnx_video.py         # Test video file
│   ├── requirements.txt            # Python dependencies for scripts
│   └── README.md                   # Scripts documentation
├── config/
└── requirements.txt
```

## Installation

### 1. Install Dependencies
```bash
pip3 install -r requirements.txt
```

### 2. Build Package
```bash
cd ~/ros2_ws
colcon build --packages-select dexi_yolo
source install/setup.bash
```

## Usage

### Hardware Deployment (Pi CM4)

```bash
# Using launch file (recommended)
ros2 launch dexi_yolo yolo_onnx_launch.py

# With custom parameters
ros2 launch dexi_yolo yolo_onnx_launch.py \
    input_size:=320 \
    num_threads:=1 \
    detection_frequency:=1.0

# Or run node directly
ros2 run dexi_yolo dexi_yolo_node_onnx.py --ros-args \
    -p confidence_threshold:=0.6 \
    -p detection_frequency:=2.0 \
    -p num_threads:=1
```

### Desktop Development

```bash
# Basic simulation
ros2 run dexi_yolo dexi_yolo_simulator

# Traffic scenario at 2Hz
ros2 run dexi_yolo dexi_yolo_simulator --ros-args \
    -p scenario:=traffic \
    -p detection_frequency:=2.0

# Available scenarios: traffic, indoor, outdoor, mixed
```

### Monitor Detections

```bash
# Listen to detection data (same format for all engines)
ros2 topic echo /yolo_detections
```

## Configuration

### ONNX Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | `models/best_optimized.onnx` | Path to ONNX model file |
| `input_size` | `320` | Model input size (320x320 for custom model) |
| `confidence_threshold` | `0.5` | Detection confidence threshold |
| `detection_frequency` | `1.0` | Detection rate (Hz) |
| `num_threads` | `2` | CPU threads (reduce for Pi, increase for desktop) |
| `nms_threshold` | `0.4` | Non-Maximum Suppression threshold |
| `use_letterbox` | `true` | Use letterbox preprocessing (preserves aspect ratio) |

### Hailo Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hef_path` | `models/best_optimized.hef` | Path to Hailo HEF model (defaults to package model) |
| `confidence_threshold` | `0.5` | Detection confidence threshold |
| `detection_frequency` | `10.0` | Max detection rate (Hz) |
| `class_names` | `car,motorcycle,truck,bird,cat,dog` | Comma-separated class names matching model output order |

### Simulator Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scenario` | `mixed` | Scene type: traffic, indoor, outdoor, mixed |
| `detection_probability` | `0.7` | Chance of detecting objects per frame |
| `max_objects_per_frame` | `3` | Maximum objects per detection |
| `enable_movement` | `true` | Objects move between frames |

## Topics

### Input
- `/cam0/image_raw/compressed` - Compressed camera images

### Output
- `/yolo_detections` - Detection results (custom `YoloDetectionArray` message type)

### Message Format

The detections are published using a custom message type from `dexi_interfaces`:

```
# YoloDetectionArray message
std_msgs/Header header
float64 timestamp
YoloDetection[] detections

# YoloDetection message
string class_name
float32 confidence
float32[] bbox  # [x1, y1, x2, y2] - normalized coordinates (0.0-1.0)
```

**Example:**
```python
header:
  stamp: {sec: 1234567890, nanosec: 123456789}
  frame_id: "camera_frame"
timestamp: 1234567890.123
detections:
  - class_name: "cat"
    confidence: 0.95
    bbox: [0.1, 0.2, 0.8, 0.9]
  - class_name: "dog"
    confidence: 0.87
    bbox: [0.3, 0.4, 0.7, 0.8]
```

**Note:** Requires the `dexi_interfaces` package for custom message types.

## Hailo 8L Model Compilation

The Hailo 8L is an AI accelerator (NPU) available as an M.2 module for the Raspberry Pi 5. It runs quantized models in HEF format at much higher throughput and lower power than CPU inference.

### Overview

The compilation pipeline converts a trained ONNX model to Hailo's HEF format:

```
ONNX (.onnx)  -->  Parse  -->  HAR  -->  Quantize (INT8)  -->  HAR  -->  Compile  -->  HEF
```

No GPU or training is involved — this is purely a format conversion and quantization step. The Hailo Dataflow Compiler (DFC) runs on CPU and uses a calibration dataset to determine optimal INT8 quantization ranges.

### Prerequisites

- **Linux x86_64** (or WSL 2 on Windows)
- **Python 3.10** (only version supported by DFC)
- **Hailo Dataflow Compiler** — download the `.whl` from [Hailo Developer Zone](https://hailo.ai/developer-zone/) (requires free account)

### Setup

```bash
# Install Python 3.10 if needed (Ubuntu)
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt install python3.10 python3.10-venv python3.10-dev
sudo apt install graphviz libgraphviz-dev python3-tk

# Create venv and install DFC
python3.10 -m venv venv
source venv/bin/activate
pip install hailo_dataflow_compiler-<version>.whl
pip install opencv-python-headless
```

### Compile

```bash
source venv/bin/activate
cd hailo
./compile_model.sh
```

The script will:
1. Download 128 class-balanced calibration images from COCO val2017
2. Parse the ONNX model into a Hailo Archive (HAR)
3. Quantize to INT8 using the calibration data
4. Compile to HEF for the Hailo 8L

Output: `models/best_optimized.hef`

> **Targeting a full Hailo 8 (26 TOPS):** `compile_model.sh` hardcodes
> `HW_ARCH="hailo8l"`. The resulting HEF runs on a Hailo 8 but won't use its extra
> capacity (HailoRT warns "lower performance"). To compile natively for the Hailo 8,
> set `HW_ARCH="hailo8"` in the script (all three `--hw-arch` steps pick it up). See
> the Hailo 8 vs 8L benchmark under [Performance](#performance).

### Deploy to Pi

The compiled `best_optimized.hef` is checked into `models/`, so deploying is just
pulling the repo and rebuilding the workspace:

```bash
cd ~/dexi_ws/src/dexi_yolo
git pull
cd ~/dexi_ws && colcon build --packages-select dexi_yolo
source install/setup.bash

# Launch (uses best_optimized.hef by default)
ros2 launch dexi_yolo yolo_hailo_launch.py
```


## Performance

### Pi CM4 Recommendations
- **Use ONNX node** for 30-50% lower CPU usage
- **Limit threads**: `num_threads:=1` for single-core performance
- **Typical performance**: ~230ms inference, ~60-80% CPU usage
- **NMS enabled**: Clean single detections vs duplicates

### Engine Comparison
| Engine | CPU Usage | Inference Time | Memory | Notes |
|--------|-----------|----------------|---------|--------|
| PyTorch | Higher | ~250ms | More | Full framework |
| ONNX (Pi CM4) | 60–80% | ~230ms | ~247 MB | CPU-only, fine on CM4 |
| ONNX (Pi 5) | ~31% | ~175ms | ~247 MB | CPU-only, measured at 2 Hz |
| Hailo 8L (Pi 5) | ~7% | ~8.9ms (~110 FPS) | ~214 MB | NPU accelerated, Pi 5 + M.2 hat only |
| Hailo 8 (Pi 5) | ~7% | ~9.0ms (~99 FPS) | ~214 MB | 26-TOPS M.2 module; running the 8L-compiled HEF gives **no speedup** — recompile with `--hw-arch hailo8` to use the bigger chip (see note below) |
| Simulator | Minimal | ~1ms | Minimal | Desktop testing |

### Pi 5: ONNX (CPU) vs Hailo 8L (Measured)

Apples-to-apples 60s inference test on identical scene, same detection rate (2 Hz),
same companion nodes running (camera, apriltag, bringup). Each engine was allowed
to settle for 60 seconds before measurement.

| Metric                | ONNX (CPU, FP32) | Hailo 8L (INT8, NPU) | Delta          |
|-----------------------|------------------|----------------------|----------------|
| Detection count       | 113              | 113                  | tie            |
| Class correctness     | 113/113 `dog`    | 113/113 `dog`        | tie            |
| False positives       | 0                | 0                    | tie            |
| Avg confidence        | 0.744            | 0.877                | **+0.133**     |
| Confidence range      | varies           | 0.856 – 0.885        | Hailo tighter  |
| Inference process CPU | ~31.2% sustained | ~7.2% sustained      | **~4.3× less** |
| Memory (RSS)          | ~247 MB          | ~214 MB              | ~33 MB less    |
| System load avg       | 1.64 – 1.91      | 1.44 – 1.69          | Hailo lower    |
| Temperature           | 67.5 – 69.2 °C   | 64.8 – 65.9 °C       | ~3 °C cooler   |
| Inference time        | ~175 ms / frame  | ~8.9 ms / frame      | **~20× faster**|
| Max throughput        | ~5.7 FPS         | ~110 FPS             | **~20× higher**|

**Takeaway:** The Hailo 8L matches ONNX on detection accuracy (identical count, zero
false positives) while using ~4× less CPU, running ~20× faster, and actually producing
*higher* average confidence (0.88 vs 0.74). On Pi 5 with the Hailo M.2 hat, the Hailo
engine is the clear choice — it frees up ~24% of CPU for other nodes (apriltag,
offboard control, etc.) while delivering better-quality detections at a much higher
throughput ceiling.

### Hailo 8 (26 TOPS) vs Hailo 8L (13 TOPS)

The Hailo M.2 module ships in two variants: the **Hailo 8L** (13 TOPS, on the
Raspberry Pi AI Kit) and the full **Hailo 8** (26 TOPS, e.g. the AI HAT+ 26T or an
M.2 8 module). Both enumerate on a Pi 5 with no `config.txt`/PCIe overlay changes —
the in-kernel `hailo_pci` driver auto-probes once `hailo-all` is installed and the
firmware blob is present (a reboot is required after install so the driver re-probes).

A HEF compiled for `hailo8l` **runs as-is on a Hailo 8** (forward-compatible), but
HailoRT prints `HEF was compiled for Hailo8L device, while the device itself is
Hailo8. This will result in lower performance.` — and the warning is literally true:
the HEF only allocates 8L-sized resources, so the bigger chip sits underutilized.

Measured on a Pi 5 + **Hailo 8** M.2 (`HAILO-8 AI ACC M.2 M KEY MODULE EXT TEMP`,
arch `HAILO8`, FW 4.20.0), using the bundled (8L-compiled) `best_optimized.hef` —
`hailortcli run`, batch size 1, 6s runs:

| Metric                | Hailo 8L (8L HEF)¹ | Hailo 8 (same 8L HEF) | Hailo 8 (native COCO `yolov8s_h8.hef`)² |
|-----------------------|--------------------|-----------------------|------------------------------------------|
| HW latency / frame    | ~8.9 ms            | **9.04 ms** (overall 10.23 ms) | 8.16 ms                        |
| Max throughput        | ~110 FPS           | **98.6 FPS**          | 151 FPS                                  |
| Power (avg)           | —                  | **0.97 W**            | —                                        |
| Chip temperature      | —                  | **~56 °C**            | —                                        |

¹ 8L column is the ROS-node measurement from the table above (same model). The 8L and
Hailo-8 numbers for the 8L HEF are within noise of each other — confirming the bigger
chip gives no benefit until the model is recompiled for it.
² Different (heavier, 80-class COCO) network compiled natively for `hailo8`, shown only
as a reference that the Hailo 8 *does* run native-arch HEFs.

**Takeaway:** Don't expect a free speedup from dropping a Hailo 8 in place of an 8L —
the existing 8L HEF runs ~identically (marginally slower). To actually use the 26-TOPS
part, recompile the model for `hailo8` (see below). For this 320×320 6-class model it
doesn't matter much in practice: both variants already exceed the 2 Hz detection
pipeline by ~50×, draw ~1 W, and stay cool (~56 °C) — the 8L is plenty. The Hailo 8
headroom only becomes useful with larger models or higher frame rates.

## Local Testing (Desktop)

Test the ONNX detection node on your desktop before deploying to Pi using a camera simulator that publishes video frames.

### Quick Local Test

```bash
# Run both camera simulator and ONNX node together
ros2 launch dexi_yolo camera_simulator_launch.py

# With custom video and settings
ros2 launch dexi_yolo camera_simulator_launch.py \
    video_path:=/path/to/your/video.mp4 \
    input_size:=320 \
    use_letterbox:=true
```

### Manual Testing (Run Nodes Separately)

```bash
# Terminal 1: Start camera simulator
ros2 run dexi_yolo camera_simulator_node.py --ros-args \
    -p video_path:=scripts/dexi_test_flight.mp4 \
    -p fps:=30.0 \
    -p loop:=true

# Terminal 2: Start ONNX detection node
ros2 run dexi_yolo dexi_yolo_node_onnx.py --ros-args \
    -p input_size:=320 \
    -p use_letterbox:=false \
    -p num_threads:=4

# Terminal 3: Monitor detections
ros2 topic echo /yolo_detections
```

### Camera Simulator Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `video_path` | `scripts/dexi_test_flight.mp4` | Path to test video |
| `fps` | `30.0` | Playback frame rate |
| `loop` | `true` | Loop video when finished |
| `compression_quality` | `90` | JPEG compression quality (0-100) |

### Benefits of Local Testing

- ✅ Test detection accuracy before Pi deployment
- ✅ Faster iteration (no deploy/boot cycle)
- ✅ Use more CPU threads for faster testing
- ✅ Validate model changes immediately
- ✅ Compare preprocessing methods (letterbox vs simple resize)

## Development Workflow

1. **Local Testing**: Use camera simulator with test video to validate detections
2. **Desktop Development**: Use detection simulator for UI/logic testing
3. **Hardware Testing (Pi CM4)**: Deploy ONNX node
4. **Hardware Testing (Pi 5 + Hailo)**: Deploy Hailo node for accelerated inference
5. **Performance Tuning**: Adjust `num_threads` (ONNX) or `detection_frequency` (all engines)
6. **Production**: Use Hailo node on Pi 5 or ONNX node on Pi CM4

## Troubleshooting

### High CPU Usage
```bash
# Reduce CPU threads
ros2 run dexi_yolo dexi_yolo_node_onnx.py --ros-args -p num_threads:=1

# Lower detection frequency  
ros2 run dexi_yolo dexi_yolo_node_onnx.py --ros-args -p detection_frequency:=0.5
```

### Multiple Detections
- NMS is enabled by default (`nms_threshold:=0.4`)
- Increase threshold for stricter filtering: `nms_threshold:=0.3`

### Simulation Testing
```bash
# Static objects (no movement)
ros2 run dexi_yolo dexi_yolo_simulator --ros-args -p enable_movement:=false

# High detection rate for testing
ros2 run dexi_yolo dexi_yolo_simulator --ros-args -p detection_probability:=1.0
```

## License

MIT License - see LICENSE file for details.
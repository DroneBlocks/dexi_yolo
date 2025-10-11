# Dexi YOLO ROS2 Package

A ROS2 package for YOLO object detection optimized for Raspberry Pi CM4. Provides both PyTorch and ONNX Runtime implementations with desktop simulation support.

## Features

- **Multiple Detection Engines**: PyTorch and optimized ONNX Runtime
- **Pi CM4 Optimized**: CPU thread limiting, memory optimizations, NMS
- **Desktop Simulation**: Test without hardware using realistic detection data
- **Offline Ready**: Models included for deployment without internet
- **Unified Interface**: All engines publish to `/yolo_detections`

## Quick Start

```bash
# Real hardware (Pi CM4) - using launch files
ros2 launch dexi_yolo yolo_onnx_launch.py  # ONNX (recommended)
ros2 launch dexi_yolo yolo_launch.py       # PyTorch

# Or run nodes directly
ros2 run dexi_yolo dexi_yolo_node_onnx.py  # ONNX
ros2 run dexi_yolo dexi_yolo_node.py       # PyTorch

# Desktop simulation
ros2 run dexi_yolo dexi_yolo_simulator
```

## Package Structure

```
dexi_yolo/
├── src/
│   ├── dexi_yolo_node.py           # PyTorch implementation
│   ├── dexi_yolo_node_onnx.py      # ONNX Runtime (optimized)
│   ├── dexi_yolo_simulator.py      # Detection simulator
│   └── camera_simulator_node.py    # Camera/video simulator for testing
├── models/
│   ├── yolov8n.pt                  # PyTorch model (6MB)
│   ├── yolov8n.onnx                # ONNX model (12MB)
│   └── best_optimized.onnx         # Custom trained model
├── launch/
│   ├── yolo_launch.py              # PyTorch node launch file
│   ├── yolo_onnx_launch.py         # ONNX node launch file (recommended)
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
    input_size:=640 \
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
| `input_size` | `640` | Model input size (640x640 for custom model) |
| `confidence_threshold` | `0.5` | Detection confidence threshold |
| `detection_frequency` | `1.0` | Detection rate (Hz) |
| `num_threads` | `2` | CPU threads (reduce for Pi, increase for desktop) |
| `nms_threshold` | `0.4` | Non-Maximum Suppression threshold |
| `use_letterbox` | `false` | Use letterbox preprocessing (preserves aspect ratio) |

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
- `/yolo_detections` - Detection results (JSON format)

### Message Format
```json
{
  "header": {
    "stamp": {"sec": 1234567890, "nanosec": 123456789},
    "frame_id": "camera_frame"
  },
  "detections": [
    {
      "class_name": "person",
      "confidence": 0.95,
      "bbox": [0.1, 0.2, 0.8, 0.9]
    }
  ],
  "timestamp": 1234567890.123,
  "engine": "onnx"
}
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
| ONNX | Lower | ~230ms | Less | Optimized runtime |
| Simulator | Minimal | ~1ms | Minimal | Desktop testing |

## Local Testing (Desktop)

Test the ONNX detection node on your desktop before deploying to Pi using a camera simulator that publishes video frames.

### Quick Local Test

```bash
# Run both camera simulator and ONNX node together
ros2 launch dexi_yolo camera_simulator_launch.py

# With custom video and settings
ros2 launch dexi_yolo camera_simulator_launch.py \
    video_path:=/path/to/your/video.mp4 \
    input_size:=640 \
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
ros2 run dexi_yolo dexi_yolo_node_onnx --ros-args \
    -p input_size:=640 \
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
3. **Hardware Testing**: Deploy ONNX node to Pi CM4
4. **Performance Tuning**: Adjust `num_threads` and `detection_frequency`
5. **Production**: Use ONNX node with optimized parameters

## Troubleshooting

### High CPU Usage
```bash
# Reduce CPU threads
ros2 run dexi_yolo dexi_yolo_node_onnx --ros-args -p num_threads:=1

# Lower detection frequency  
ros2 run dexi_yolo dexi_yolo_node_onnx --ros-args -p detection_frequency:=0.5
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
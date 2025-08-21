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
# Real hardware (Pi CM4)
ros2 run dexi_yolo dexi_yolo_node_onnx     # ONNX (recommended)
ros2 run dexi_yolo dexi_yolo_node          # PyTorch

# Desktop simulation
ros2 run dexi_yolo dexi_yolo_simulator
```

## Package Structure

```
dexi_yolo/
├── src/
│   ├── dexi_yolo_node.py          # PyTorch implementation
│   ├── dexi_yolo_node_onnx.py     # ONNX Runtime (optimized)
│   └── dexi_yolo_simulator.py     # Desktop simulator
├── models/
│   ├── yolov8n.pt                 # PyTorch model (6MB)
│   └── yolov8n.onnx               # ONNX model (12MB)
├── launch/
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
# Optimized ONNX (recommended - lower CPU usage)
ros2 run dexi_yolo dexi_yolo_node_onnx

# With custom parameters
ros2 run dexi_yolo dexi_yolo_node_onnx --ros-args \
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
| `confidence_threshold` | `0.5` | Detection confidence threshold |
| `detection_frequency` | `1.0` | Detection rate (Hz) |
| `num_threads` | `2` | CPU threads (reduce for lower usage) |
| `nms_threshold` | `0.4` | Non-Maximum Suppression threshold |
| `input_size` | `320` | Model input size (320x320) |

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

## Development Workflow

1. **Desktop Development**: Use simulator for UI/logic testing
2. **Hardware Testing**: Deploy ONNX node to Pi CM4
3. **Performance Tuning**: Adjust `num_threads` and `detection_frequency`
4. **Production**: Use ONNX node with optimized parameters

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
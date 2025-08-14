# Dexi YOLO ROS2 Package

A ROS2 package that provides YOLO object detection capabilities by subscribing to compressed images and publishing detection results. Designed for offline deployment on Raspberry Pi4 and other embedded systems.

## Features

- Subscribes to `/cam0/image_raw/compressed` topic for input images
- Publishes detection results to `/yolo_detections` topic
- Configurable confidence threshold and detection frequency
- Support for different YOLO model sizes (nano, small, medium, large)
- Efficient image processing with frequency control
- JSON-formatted detection output with bounding box coordinates
- **Offline-ready**: Models included in package for Pi4 deployment without internet access

## Package Structure

```
dexi_yolo/
├── src/                        # Source code
│   └── dexi_yolo_node.py      # Main YOLO detection node
├── models/                     # YOLO model files (offline deployment)
│   ├── README.md              # Model documentation
│   ├── yolov8n.pt             # YOLOv8 Nano (recommended for Pi4)
│   └── yolov8s.pt             # YOLOv8 Small (balanced)
├── launch/                     # Launch files
│   └── yolo_launch.py         # Node launcher
├── config/                     # Configuration files
│   └── dexi_yolo_params.yaml  # Default parameters
├── resource/                   # ROS2 resource marker
├── test/                       # Test scripts
├── demo/                       # Demo scripts
├── package.xml                # Package manifest
├── CMakeLists.txt            # Build configuration
├── requirements.txt           # Python dependencies
└── README.md                 # This file
```

## Installation

### Prerequisites

- ROS2 Humble or later
- Python 3.8+
- Raspberry Pi4 (recommended) or other ARM64/x86 system
- CUDA-capable GPU (optional, for faster inference)

### 1. Install Dependencies

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install ros-humble-cv-bridge python3-opencv

# Install Python dependencies
pip3 install -r requirements.txt
```

### 2. Add YOLO Models (Required for Offline Use)

Before building, download YOLO models to the `models/` directory:

```bash
# Navigate to the models directory
cd models/

# Download recommended models for Pi4
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.pt

# Or manually download from: https://github.com/ultralytics/assets/releases
```

### 3. Build the Package

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws

# Copy this package to src/
cp -r /path/to/dexi_yolo src/

# Build the package
colcon build --packages-select dexi_yolo

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Usage

```bash
# Launch the node with default parameters (uses models/yolov8n.pt)
ros2 launch dexi_yolo yolo_launch.py

# Or run the node directly
ros2 run dexi_yolo dexi_yolo_node
```

### Custom Parameters

```bash
# Launch with custom model
ros2 launch dexi_yolo yolo_launch.py \
    model_path:=models/yolov8s.pt \
    confidence_threshold:=0.7 \
    detection_frequency:=2.0

# Use different model
ros2 launch dexi_yolo yolo_launch.py model_path:=models/yolov8m.pt
```

### Command Line Parameters

```bash
# Run with command line arguments
ros2 run dexi_yolo dexi_yolo_node \
    --ros-args \
    -p model_path:=models/yolov8s.pt \
    -p confidence_threshold:=0.6 \
    -p detection_frequency:=1.5
```

## Configuration

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | `models/yolov8n.pt` | Path to YOLO model file in package |
| `confidence_threshold` | `0.5` | Confidence threshold (0.0-1.0) |
| `detection_frequency` | `1.0` | Detection frequency in Hz |

### Model Options for Pi4

- `models/yolov8n.pt` - **Nano (6MB)**: Fastest, lowest memory, recommended for Pi4
- `models/yolov8s.pt` - **Small (22MB)**: Balanced speed/accuracy, good for Pi4
- `models/yolov8m.pt` - **Medium (52MB)**: Higher accuracy, slower on Pi4
- `models/yolov8l.pt` - **Large (87MB)**: Highest accuracy, may be slow on Pi4

## Topics

### Subscribed Topics

- `/cam0/image_raw/compressed` - Input compressed images

### Published Topics

- `/yolo_detections` - Detection results (std_msgs/String)

### Message Format

The detection results are published as JSON strings containing:

```json
{
  "header": {
    "stamp": {
      "sec": 1234567890,
      "nanosec": 123456789
    },
    "frame_id": "camera_frame"
  },
  "detections": [
    {
      "class_name": "person",
      "confidence": 0.95,
      "bbox": [0.1, 0.2, 0.8, 0.9]
    }
  ],
  "timestamp": 1234567890.123
}
```

Where `bbox` contains normalized coordinates `[x1, y1, x2, y2]` (0.0-1.0).

## Testing

### Test with Sample Images

```bash
# Publish a test image (if you have one)
ros2 run image_transport republish raw compressed -- /test_image

# Monitor detections
ros2 topic echo /yolo_detections
```

### Monitor Node Status

```bash
# Check node status
ros2 node list
ros2 node info /dexi_yolo_node

# View node logs
ros2 run dexi_yolo dexi_yolo_node --ros-args --log-level DEBUG
```

## Pi4 Deployment

### Offline Setup

1. **Include models in package**: Models are automatically installed to the package's share directory
2. **No internet required**: Package works completely offline after build
3. **Optimized for ARM64**: YOLO models run efficiently on Pi4's ARM processor

### Performance Tips for Pi4

- Use `yolov8n.pt` for real-time applications (30+ FPS possible)
- Set `detection_frequency` to 1-2 Hz for balanced performance
- Monitor Pi4 temperature during extended use
- Consider using a heatsink for sustained operation

## Troubleshooting

### Common Issues

1. **Model not found**: Ensure YOLO model files are in the `models/` directory before building
2. **Import errors**: Make sure all Python dependencies are installed
3. **Performance issues**: Use smaller models (nano/small) on Pi4
4. **Memory issues**: YOLO models can be memory-intensive; ensure sufficient RAM (4GB+ recommended)

### Debug Mode

```bash
# Run with debug logging
ros2 run dexi_yolo dexi_yolo_node --ros-args --log-level DEBUG
```

## Performance Tips

- Use `yolov8n.pt` for real-time applications on Pi4
- Adjust `detection_frequency` based on your needs and Pi4 performance
- Monitor system resources during operation
- Consider using a USB3 SSD for faster model loading

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly on target hardware
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For issues and questions:
- Check the troubleshooting section
- Review ROS2 documentation
- Open an issue on the repository

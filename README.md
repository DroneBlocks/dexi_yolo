# DEXI YOLO ROS2 Package

A ROS2 package that provides YOLO object detection capabilities by subscribing to compressed images and publishing detection results.

## Features

- Subscribes to `/cam0/image_raw/compressed` topic for input images
- Publishes detection results to `/yolo_detections` topic
- Configurable confidence threshold and detection frequency
- Support for different YOLO model sizes (nano, small, medium, large)
- Efficient image processing with frequency control
- JSON-formatted detection output with bounding box coordinates

## Package Structure

```
dexi_yolo/
├── src/                        # Source code
│   └── dexi_yolo_node.py      # Main YOLO detection node
├── launch/                     # Launch files
│   └── dexi_yolo_launch.py    # Node launcher
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
- CUDA-capable GPU (optional, for faster inference)

### 1. Install Dependencies

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install ros-humble-cv-bridge python3-opencv

# Install Python dependencies
pip3 install -r requirements.txt
```

### 2. Build the Package

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
# Launch the node with default parameters
ros2 launch dexi_yolo dexi_yolo_launch.py

# Or run the node directly
ros2 run dexi_yolo dexi_yolo_node
```

### Custom Parameters

```bash
# Launch with custom parameters
ros2 launch dexi_yolo dexi_yolo_launch.py \
    model_path:=yolov8s.pt \
    confidence_threshold:=0.7 \
    detection_frequency:=2.0
```

### Command Line Parameters

```bash
# Run with command line arguments
ros2 run dexi_yolo dexi_yolo_node \
    --ros-args \
    -p model_path:=yolov8m.pt \
    -p confidence_threshold:=0.6 \
    -p detection_frequency:=1.5
```

## Configuration

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | `yolov8n.pt` | Path to YOLO model file |
| `confidence_threshold` | `0.5` | Confidence threshold (0.0-1.0) |
| `detection_frequency` | `1.0` | Detection frequency in Hz |

### Model Options

- `yolov8n.pt` - Nano (fastest, least accurate)
- `yolov8s.pt` - Small (balanced)
- `yolov8m.pt` - Medium (more accurate)
- `yolov8l.pt` - Large (most accurate, slowest)

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

## Troubleshooting

### Common Issues

1. **Model not found**: Ensure the YOLO model file exists at the specified path
2. **Import errors**: Make sure all Python dependencies are installed
3. **Performance issues**: Consider using a smaller model or adjusting detection frequency
4. **Memory issues**: YOLO models can be memory-intensive; ensure sufficient RAM

### Debug Mode

```bash
# Run with debug logging
ros2 run dexi_yolo dexi_yolo_node --ros-args --log-level DEBUG
```

## Performance Tips

- Use `yolov8n.pt` for real-time applications
- Adjust `detection_frequency` based on your needs
- Consider using GPU acceleration if available
- Monitor system resources during operation

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For issues and questions:
- Check the troubleshooting section
- Review ROS2 documentation
- Open an issue on the repository

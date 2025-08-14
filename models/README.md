# Models Directory

This directory contains YOLO model files for offline deployment on the Raspberry Pi4.

## Adding Models

1. **Download YOLO models** from the official repository:
   - [YOLOv8 Models](https://github.com/ultralytics/assets/releases)
   - Recommended for Pi4: `yolov8n.pt` (nano) or `yolov8s.pt` (small)

2. **Place models here** before building the package:
   ```
   models/
   ├── README.md
   ├── yolov8n.pt          # YOLOv8 Nano (fastest, ~6MB)
   ├── yolov8s.pt          # YOLOv8 Small (balanced, ~22MB)
   └── yolov8m.pt          # YOLOv8 Medium (accurate, ~52MB)
   ```

## Model Selection for Pi4

- **yolov8n.pt**: Best for real-time applications, lowest memory usage
- **yolov8s.pt**: Good balance of speed and accuracy
- **yolov8m.pt**: Higher accuracy but slower inference

## Package Configuration

The default model path in the package is `models/yolov8n.pt`. You can change this:

```bash
# Launch with custom model
ros2 launch dexi_yolo yolo_launch.py model_path:=models/yolov8s.pt

# Or run with parameter
ros2 run dexi_yolo dexi_yolo_node --ros-args -p model_path:=models/yolov8m.pt
```

## File Sizes

- yolov8n.pt: ~6 MB
- yolov8s.pt: ~22 MB  
- yolov8m.pt: ~52 MB
- yolov8l.pt: ~87 MB

## Note

Models are automatically installed to the package's share directory during build, making them available at runtime without internet access.

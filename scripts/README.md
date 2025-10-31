# Debug Scripts

Standalone Python scripts for debugging ONNX model inference outside of ROS.

## Setup

Create a virtual environment and install dependencies:

```bash
cd scripts
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

## Scripts

### debug_onnx_inference.py

Diagnostic tool to test ONNX model inference on a single image and identify preprocessing/postprocessing issues.

**Usage:**

```bash
# Place a test image in the project root
cp /path/to/test_image.jpg ../test_image.jpg

# Run the diagnostic
python debug_onnx_inference.py
```

**What it does:**
- Inspects model input/output shapes
- Tests two preprocessing methods (simple resize vs letterbox)
- Analyzes raw model outputs
- Identifies if sigmoid activation is needed
- Shows confidence scores and detections

**Configuration:**

Edit these variables in the script:
```python
model_path = "models/best_optimized.onnx"  # Your model path
test_image = "test_image.jpg"               # Your test image
class_names = ['car', 'motorcycle', 'truck', 'bird', 'cat', 'dog']  # Your classes
```

---

### debug_onnx_video.py

Video processing tool for testing ONNX model inference on MP4 drone footage. Processes frames, shows detections, and saves annotated output video.

**Usage:**

```bash
# Place your test video in the project root
cp /path/to/drone_footage.mp4 ../test_video.mp4

# Run the video processor
python debug_onnx_video.py
```

**What it does:**
- Processes MP4 video frame by frame
- Tests inference on real drone footage
- Draws bounding boxes and labels on frames
- Saves annotated output video
- Provides detailed statistics (detection rate, timing, class distribution)
- Supports both preprocessing methods (simple resize vs letterbox)

**Configuration:**

Edit these variables in the script:
```python
video_path = "../test_video.mp4"            # Your MP4 file
model_path = "../models/best_optimized.onnx"
class_names = ['car', 'motorcycle', 'truck', 'bird', 'cat', 'dog']
input_size = 320                            # Model trained at 320x320
conf_threshold = 0.5                        # Lower if needed (0.25)
use_letterbox = False                       # True for letterbox padding
max_frames = None                           # Limit frames for testing (e.g., 100)
```

**Output:**

Creates an annotated video file: `test_video_output_resize.mp4` (or `_letterbox.mp4`)

**Tips:**
- Start with `max_frames = 100` to quickly test settings
- If no detections: lower `conf_threshold` to 0.25 or try different `input_size`
- If boxes are misaligned: try `use_letterbox = True`
- Compare results with: `yolo predict source=test_video.mp4 model=models/best_optimized.onnx`

## Troubleshooting

If you get import errors, make sure you're in the virtual environment:
```bash
source venv/bin/activate
```

If opencv-python fails to install, try:
```bash
pip install --upgrade pip
pip install opencv-python-headless
```

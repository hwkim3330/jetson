---
layout: default
title: AI Features
---

# AI Features

This guide covers the AI and computer vision capabilities of the SDV platform.

## Overview

SDV uses TensorRT-accelerated YOLOv8 for real-time object detection on the Jetson Orin Nano.

## YOLOv8 Object Detection

### Performance

| Metric | Value |
|--------|-------|
| Model | YOLOv8n (nano) |
| Precision | FP16 |
| Engine Size | 9 MB |
| Throughput | **147 FPS** |
| Latency | 7.6 ms |
| GPU Compute | 6.8 ms |

### Using YOLO

1. Go to **AI** tab in Web UI
2. Press **YOLO** button
3. View detections overlaid on camera
4. Detection log shows class and confidence

### Detection Classes

YOLOv8n detects 80 COCO classes including:

| Category | Classes |
|----------|---------|
| People | person |
| Vehicles | car, truck, bus, motorcycle, bicycle |
| Animals | dog, cat, bird, horse |
| Objects | chair, couch, tv, laptop, phone |

### Bounding Box Colors

| Color | Class |
|-------|-------|
| Green | Person |
| Orange | Other objects |

## TensorRT Engine

### Building the Engine

```bash
cd ~/ros2_ws/src/robot_ai/models

# Download YOLOv8n
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"

# Export to ONNX
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='onnx')"

# Build TensorRT FP16 engine
/usr/src/tensorrt/bin/trtexec \
    --onnx=yolov8n.onnx \
    --saveEngine=yolov8n_fp16.engine \
    --fp16
```

### Engine Search Paths

The YOLO detector searches for models in this order:

1. Parameter: `model_path`
2. Default: `~/ros2_ws/src/robot_ai/models/yolov8n_fp16.engine`
3. Fallback: `~/ros2_ws/src/robot_ai/models/yolov8n.pt`
4. Download: Automatic from Ultralytics

## Topics

### Input

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw/compressed` | CompressedImage | Camera feed |

### Output

| Topic | Type | Description |
|-------|------|-------------|
| `/detections` | String | JSON array of detections |
| `/person_detected` | Bool | True if person found |
| `/person_distance` | Float32 | Estimated distance |
| `/person_target` | Point | Target position (x,y normalized) |

### Detection Format

`/detections` JSON format:
```json
[
  {
    "class": "person",
    "confidence": 0.89,
    "bbox": [x1, y1, x2, y2]
  },
  {
    "class": "chair",
    "confidence": 0.75,
    "bbox": [x1, y1, x2, y2]
  }
]
```

## Control Topics

### Enable YOLO

```bash
ros2 topic pub /robot/ai_mode std_msgs/String "data: 'yolo'"
```

### Disable YOLO

```bash
ros2 topic pub /robot/ai_mode std_msgs/String "data: 'off'"
```

## Code Reference

### yolo_detector.py

Location: `src/robot_ai/scripts/yolo_detector.py`

Key functionality:
```python
class YoloDetector(Node):
    def __init__(self):
        # Load TensorRT engine
        self.model = YOLO(engine_path)

    def image_callback(self, msg):
        # Decompress JPEG
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Run inference
        results = self.model(frame)

        # Publish detections
        self.publish_detections(results)
```

## GPU Memory Usage

| Component | Memory |
|-----------|--------|
| TensorRT Engine | ~8 MB |
| Runtime Allocation | ~300 MB |
| Total | ~308 MB |

**Note**: Disable YOLO when not needed to free GPU memory.

## Other AI Modes (Experimental)

These modes are available but may require tuning:

| Mode | Command | Description |
|------|---------|-------------|
| Body Tracker | `body` | Track body pose |
| Gesture | `gesture` | Gesture recognition |
| Person Follower | `person` | Follow detected person |
| Multi AI | `multi` | Multiple detectors |

### Activate AI Mode

```bash
ros2 topic pub /robot/ai_mode std_msgs/String "data: '<mode>'"
```

## Troubleshooting

### YOLO Not Running

```bash
# Check if detector is running
ros2 node list | grep yolo

# Check detections topic
ros2 topic echo /detections
```

### No Detections

- Verify camera is publishing: `ros2 topic hz /camera/image_raw/compressed`
- Check model file exists
- View logs: `journalctl -u robot.service -f`

### Slow Performance

- Ensure TensorRT engine is used (not .pt)
- Check GPU utilization: `tegrastats`
- Reduce camera resolution if needed

### Out of Memory

- Disable YOLO when not needed
- Reduce batch size
- Use smaller model (yolov8n)

## Best Practices

1. **Use TensorRT**: Always use `.engine` file for best performance
2. **FP16 Precision**: Provides good accuracy with 2x speed
3. **Disable When Idle**: Turn off YOLO to save GPU memory
4. **Monitor GPU**: Use `tegrastats` to check utilization

---

[Back to Home](index.md) | [Previous: SLAM & Navigation](slam-navigation.md) | [Next: API Reference](api-reference.md)

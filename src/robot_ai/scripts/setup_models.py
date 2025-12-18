#!/usr/bin/env python3
"""
AI Model Setup Script for KETI Robot
Downloads and converts models for optimal Jetson performance:
- YOLOv8n -> TensorRT (GPU/DLA)
- MobileNet-SSD -> TensorRT
- MediaPipe models (if compatible)

Usage:
    python3 setup_models.py [--all|--yolo|--mobilenet|--mediapipe]
"""

import os
import sys
import subprocess
import urllib.request
import shutil

MODELS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS_PATH = os.path.join(MODELS_DIR, 'models')
TRT_PATH = os.path.join(MODELS_PATH, 'tensorrt')

# Model URLs
MODELS = {
    'mobilenet_ssd': {
        'prototxt': 'https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/deploy.prototxt',
        'caffemodel': 'https://drive.google.com/uc?export=download&id=0B3gersZ2cHIxRm5PMWRoTkdHdHc',
        'local_name': 'MobileNetSSD_deploy'
    },
    'yolov8n': {
        'url': 'https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt',
        'local_name': 'yolov8n.pt'
    }
}


def download_file(url, dest_path, desc="Downloading"):
    """Download file with progress"""
    print(f"{desc}: {url}")
    try:
        urllib.request.urlretrieve(url, dest_path)
        print(f"  Saved to: {dest_path}")
        return True
    except Exception as e:
        print(f"  Failed: {e}")
        return False


def setup_mobilenet_ssd():
    """Download MobileNet-SSD model"""
    print("\n=== Setting up MobileNet-SSD ===")
    os.makedirs(MODELS_PATH, exist_ok=True)

    prototxt_path = os.path.join(MODELS_PATH, 'MobileNetSSD_deploy.prototxt')
    caffemodel_path = os.path.join(MODELS_PATH, 'MobileNetSSD_deploy.caffemodel')

    # Download prototxt
    if not os.path.exists(prototxt_path):
        download_file(MODELS['mobilenet_ssd']['prototxt'], prototxt_path, "Downloading prototxt")
    else:
        print(f"Prototxt already exists: {prototxt_path}")

    # Caffemodel needs manual download due to Google Drive
    if not os.path.exists(caffemodel_path):
        print(f"\nMobileNet-SSD caffemodel needs manual download:")
        print(f"1. Download from: https://github.com/chuanqi305/MobileNet-SSD")
        print(f"2. Or use gdown: pip3 install gdown && gdown 0B3gersZ2cHIxRm5PMWRoTkdHdHc")
        print(f"3. Save to: {caffemodel_path}")
    else:
        print(f"Caffemodel already exists: {caffemodel_path}")

    return os.path.exists(prototxt_path)


def setup_yolov8():
    """Download and convert YOLOv8n to TensorRT"""
    print("\n=== Setting up YOLOv8n ===")
    os.makedirs(TRT_PATH, exist_ok=True)

    pt_path = os.path.join(MODELS_PATH, 'yolov8n.pt')
    onnx_path = os.path.join(MODELS_PATH, 'yolov8n.onnx')
    trt_path = os.path.join(TRT_PATH, 'yolov8n.engine')

    # Check if ultralytics is installed
    try:
        from ultralytics import YOLO
        print("Ultralytics installed")
    except ImportError:
        print("Installing ultralytics...")
        subprocess.run([sys.executable, '-m', 'pip', 'install', 'ultralytics', '-q'])
        try:
            from ultralytics import YOLO
        except ImportError:
            print("Failed to install ultralytics. Please install manually:")
            print("  pip3 install ultralytics")
            return False

    # Download model
    if not os.path.exists(pt_path):
        print("Downloading YOLOv8n...")
        model = YOLO('yolov8n.pt')
        shutil.move('yolov8n.pt', pt_path)
    else:
        print(f"YOLOv8n already exists: {pt_path}")

    # Export to TensorRT
    if not os.path.exists(trt_path):
        print("Converting to TensorRT (this may take several minutes)...")
        try:
            model = YOLO(pt_path)
            # Export with INT8 for DLA compatibility
            model.export(
                format='engine',
                imgsz=640,
                half=True,  # FP16
                device=0,
                workspace=4,  # GB
                verbose=False
            )
            # Move to models directory
            engine_file = pt_path.replace('.pt', '.engine')
            if os.path.exists(engine_file):
                shutil.move(engine_file, trt_path)
                print(f"TensorRT engine saved: {trt_path}")
        except Exception as e:
            print(f"TensorRT conversion failed: {e}")
            print("You can convert manually later with:")
            print(f"  yolo export model={pt_path} format=engine half=True")
            return False
    else:
        print(f"TensorRT engine already exists: {trt_path}")

    return True


def setup_mediapipe():
    """Setup MediaPipe for Jetson"""
    print("\n=== Setting up MediaPipe ===")

    # Check if mediapipe is installed
    try:
        import mediapipe as mp
        print(f"MediaPipe already installed: {mp.__version__}")
        return True
    except ImportError:
        pass

    print("Installing MediaPipe for Jetson...")
    print("Note: MediaPipe on Jetson requires special build")

    # Try pip install first
    result = subprocess.run(
        [sys.executable, '-m', 'pip', 'install', 'mediapipe', '-q'],
        capture_output=True
    )

    if result.returncode == 0:
        try:
            import mediapipe as mp
            print(f"MediaPipe installed: {mp.__version__}")
            return True
        except ImportError:
            pass

    print("\nMediaPipe pip install may not work on Jetson ARM64.")
    print("Alternative options:")
    print("1. Use pre-built wheel from NVIDIA forums")
    print("2. Build from source: https://google.github.io/mediapipe/getting_started/install.html")
    print("3. Use OpenCV DNN as fallback (already implemented)")

    return False


def create_model_info():
    """Create model info file"""
    info_path = os.path.join(MODELS_PATH, 'MODEL_INFO.md')
    info_content = """# KETI Robot AI Models

## Available Models

### 1. MobileNet-SSD (Person Detection)
- **Format**: Caffe (prototxt + caffemodel)
- **Backend**: OpenCV DNN with CUDA
- **Input Size**: 300x300
- **Classes**: 20 (VOC classes, person = class 15)
- **Speed**: ~30 FPS on Jetson Orin Nano

### 2. YOLOv8n (Object Detection)
- **Format**: TensorRT Engine (.engine)
- **Backend**: TensorRT with FP16/DLA
- **Input Size**: 640x640
- **Classes**: 80 (COCO classes)
- **Speed**: ~45 FPS on Jetson Orin Nano (FP16)

### 3. HOG Detector (Fallback)
- **Format**: OpenCV built-in
- **Backend**: CPU
- **Use Case**: When no GPU models available

## TensorRT Optimization

TensorRT engines are hardware-specific. If you get errors, regenerate:

```bash
# YOLOv8n to TensorRT
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='engine', half=True)"
```

## DLA Usage

To use DLA (Deep Learning Accelerator):
- Models must be INT8 or FP16
- Not all layers are DLA-compatible
- Fallback to GPU for unsupported layers

## Memory Usage

| Model | GPU Memory | Inference |
|-------|-----------|-----------|
| MobileNet-SSD | ~200MB | ~15ms |
| YOLOv8n (FP16) | ~400MB | ~22ms |
| YOLOv8n (INT8) | ~250MB | ~18ms |

## Adding New Models

1. Place model files in this directory
2. Update the detector node to load the model
3. For TensorRT, place .engine files in `tensorrt/` subdirectory
"""

    with open(info_path, 'w') as f:
        f.write(info_content)
    print(f"\nModel info saved: {info_path}")


def main():
    print("=" * 50)
    print("KETI Robot AI Model Setup")
    print("=" * 50)

    os.makedirs(MODELS_PATH, exist_ok=True)
    os.makedirs(TRT_PATH, exist_ok=True)

    args = sys.argv[1:] if len(sys.argv) > 1 else ['--all']

    if '--all' in args or '--mobilenet' in args:
        setup_mobilenet_ssd()

    if '--all' in args or '--yolo' in args:
        setup_yolov8()

    if '--all' in args or '--mediapipe' in args:
        setup_mediapipe()

    create_model_info()

    print("\n" + "=" * 50)
    print("Setup complete!")
    print("=" * 50)
    print(f"\nModels directory: {MODELS_PATH}")
    print(f"TensorRT directory: {TRT_PATH}")
    print("\nTo test models:")
    print("  ros2 run robot_ai person_detector.py")


if __name__ == '__main__':
    main()

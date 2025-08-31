# Iravath Perception Workspace

This repository contains perception capabilities for the Iravath robot, now converted to ROS2 Jazzy.

## Repository Structure

```
├── src/iravath_perception/          # Main ROS2 package
│   ├── scripts/                     # ROS2 node executables
│   ├── launch/                      # Launch files
│   ├── config/                      # Configuration files
│   ├── legacy_standalone/           # Original standalone scripts
│   └── ...                         # Package files
├── test.py                         # Original YOLO test script
├── test1.py                        # Original YOLO test script with measurements
└── object_dist_find/               # Original Mask R-CNN implementation
```

## Features

- **ROS2 Jazzy Compatible**: Full ROS2 integration with proper nodes, topics, and services
- **Intel RealSense Support**: Camera integration with depth and color streams
- **Multiple Detection Options**:
  - Mask R-CNN for instance segmentation
  - YOLOv5 for real-time object detection
- **3D Object Localization**: Distance measurement and real-world size estimation
- **Configurable Parameters**: Easy configuration through YAML files
- **Debug Visualization**: Real-time visualization of detection results

## Quick Start

### Prerequisites

1. **ROS2 Jazzy** installed
2. **Python dependencies**:
   ```bash
   pip install opencv-python numpy torch torchvision pyrealsense2
   ```

### Installation

1. **Clone and build**:
   ```bash
   cd ~/ros2_ws/src
   git clone <this-repository>
   cd ~/ros2_ws
   colcon build --packages-select iravath_perception
   source install/setup.bash
   ```

### Usage

#### Option 1: Launch Complete Perception System
```bash
# With Mask R-CNN (default)
ros2 launch iravath_perception perception.launch.py

# With YOLOv5
ros2 launch iravath_perception perception.launch.py use_yolo:=true
```

#### Option 2: Launch Individual Components
```bash
# Camera only
ros2 launch iravath_perception camera.launch.py

# Add object detection
ros2 run iravath_perception object_detection_node.py

# Or YOLO detection
ros2 run iravath_perception yolo_detection_node.py
```

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | sensor_msgs/Image | RGB camera feed |
| `/camera/depth/image_raw` | sensor_msgs/Image | Depth camera feed |
| `/camera/camera_info` | sensor_msgs/CameraInfo | Camera calibration |
| `/detections` | vision_msgs/Detection2DArray | Mask R-CNN results |
| `/yolo_detections` | vision_msgs/Detection2DArray | YOLO results |
| `/debug/detected_objects` | sensor_msgs/Image | Visualization |

## Configuration

Edit `src/iravath_perception/config/perception_params.yaml` to customize:

- Camera resolution and frame rate
- Detection confidence thresholds
- Model paths and parameters
- Hardware acceleration settings

## Legacy Standalone Scripts

The original standalone Python scripts are preserved in:
- `src/iravath_perception/legacy_standalone/` - Original implementations
- `test.py` and `test1.py` - Original YOLO scripts
- `object_dist_find/` - Original Mask R-CNN system

These can still be run independently without ROS2 if needed.

## Troubleshooting

1. **Camera not detected**: Ensure RealSense camera is connected and drivers installed
2. **Model files missing**: For Mask R-CNN, ensure DNN model files are in the correct paths
3. **YOLO download**: First run requires internet for model download
4. **CUDA errors**: Set `use_cuda: false` in config if CUDA is not available

## License

Apache License 2.0
# Iravath Perception Package

This package provides perception capabilities for the Iravath robot using ROS2 Jazzy.

## Features

- Intel RealSense camera integration
- Object detection using Mask R-CNN and YOLOv5
- Distance measurement and 3D object localization
- ROS2 topic-based communication
- Configurable detection parameters

## Dependencies

- ROS2 Jazzy
- Python 3.8+
- OpenCV
- PyTorch
- pyrealsense2
- numpy

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

2. Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select iravath_perception
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Launch the perception system:
```bash
ros2 launch iravath_perception perception.launch.py
```

### Individual nodes:

#### RealSense Camera Node:
```bash
ros2 run iravath_perception realsense_camera_node.py
```

#### Object Detection Node (Mask R-CNN):
```bash
ros2 run iravath_perception object_detection_node.py
```

#### YOLO Detection Node:
```bash
ros2 run iravath_perception yolo_detection_node.py
```

## Topics

- `/camera/color/image_raw` - RGB camera images
- `/camera/depth/image_raw` - Depth camera images
- `/camera/camera_info` - Camera calibration info
- `/detections` - Object detection results
- `/detected_objects` - Detected objects with 3D positions

## Parameters

Configuration parameters can be found in `config/perception_params.yaml`.

## License

Apache License 2.0
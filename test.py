import cv2
import numpy as np
import pyrealsense2 as rs
import torch  # For using a PyTorch-based object detection model

# 1. Initialize RealSense Camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# 2. Load Object Detection Model (e.g., YOLOv5, Faster R-CNN)
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Example

# 3. Main Loop
while True:
    # Get frames from camera
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        continue

    # Convert frames to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # 4. Object Detection
    results = model(color_image)

    # 5. Measurement Logic (for each detected object)
    for obj in results.xyxy[0]:
        x1, y1, x2, y2, conf, cls = obj
        if conf > 0.5:  # Filter for confidence
            # Calculate object center (in pixels)
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2

            # Get depth at object center (in meters)
            depth = depth_frame.get_distance(int(center_x), int(center_y))
            
            # Simple dimension estimation (assuming object is perpendicular to camera)
            obj_width_pixels = x2 - x1
            obj_height_pixels = y2 - y1

            # You'll need camera parameters to convert from pixels to meters here
            # ... (focal length, etc.)

            # Visualize results
            cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(color_image, f"Width: {obj_width_pixels:.2f} pixels", (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(color_image, f"Height: {obj_height_pixels:.2f} pixels", (int(x1), int(y1) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display images 
    cv2.imshow('RealSense', color_image)
    cv2.waitKey(1)

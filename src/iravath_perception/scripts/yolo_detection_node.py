#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        
        # Declare parameters
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('model_name', 'yolov5s')
        self.declare_parameter('device', 'cpu')  # 'cpu' or 'cuda'
        self.declare_parameter('focal_length', 600.0)
        
        # Get parameters
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.focal_length = self.get_parameter('focal_length').get_parameter_value().double_value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize YOLO model
        self.setup_model()
        
        # Create subscribers
        self.color_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        
        # Create publishers
        self.detections_pub = self.create_publisher(Detection2DArray, '/yolo_detections', 10)
        self.debug_image_pub = self.create_publisher(Image, '/debug/yolo_detected_objects', 10)
        
        # Store latest frames
        self.latest_color_image = None
        self.latest_depth_image = None
        
        self.get_logger().info(f'YOLO detection node started with model: {self.model_name}')

    def setup_model(self):
        """Initialize YOLO model"""
        try:
            # Load YOLOv5 model from torch hub
            self.model = torch.hub.load('ultralytics/yolov5', self.model_name, pretrained=True)
            self.model.to(self.device)
            
            # Set confidence threshold
            self.model.conf = self.confidence_threshold
            
            # Get class names
            self.class_names = self.model.names
            
            self.get_logger().info(f"Loaded YOLOv5 model on {self.device}")
            self.get_logger().info(f"Model supports {len(self.class_names)} classes")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize YOLO model: {str(e)}")
            self.get_logger().error("Make sure you have internet connection for initial model download")
            raise

    def color_callback(self, msg):
        """Handle color image messages"""
        try:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process detection
            self.process_detection(msg.header)
                
        except Exception as e:
            self.get_logger().error(f"Error processing color image: {str(e)}")

    def depth_callback(self, msg):
        """Handle depth image messages"""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")

    def process_detection(self, header):
        """Process object detection on the latest frames"""
        if self.latest_color_image is None:
            return
        
        try:
            # Run YOLO inference
            results = self.model(self.latest_color_image)
            
            # Create Detection2DArray message
            detection_array = Detection2DArray()
            detection_array.header = header
            
            debug_image = self.latest_color_image.copy()
            
            # Process detections
            for detection in results.xyxy[0]:  # xyxy format: [x1, y1, x2, y2, conf, class]
                x1, y1, x2, y2, conf, cls = detection
                
                if conf < self.confidence_threshold:
                    continue
                
                # Calculate center and dimensions
                center_x = float((x1 + x2) / 2)
                center_y = float((y1 + y2) / 2)
                width = float(x2 - x1)
                height = float(y2 - y1)
                
                # Get depth information if available
                depth_value = None
                obj_width_cm = None
                obj_height_cm = None
                
                if self.latest_depth_image is not None:
                    depth_value = self.latest_depth_image[int(center_y), int(center_x)]
                    
                    # Calculate real-world dimensions
                    if depth_value > 0:
                        depth_cm = depth_value / 10.0  # Convert to cm
                        obj_width_cm = (width * depth_cm) / self.focal_length
                        obj_height_cm = (height * depth_cm) / self.focal_length
                
                # Create Detection2D message
                detection_msg = Detection2D()
                detection_msg.header = header
                
                # Set bounding box
                detection_msg.bbox.center = Point()
                detection_msg.bbox.center.x = center_x
                detection_msg.bbox.center.y = center_y
                detection_msg.bbox.size_x = width
                detection_msg.bbox.size_y = height
                
                # Set object hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(cls))
                hypothesis.hypothesis.score = float(conf)
                detection_msg.results.append(hypothesis)
                
                detection_array.detections.append(detection_msg)
                
                # Draw on debug image
                detection_info = {
                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                    'center': (int(center_x), int(center_y)),
                    'class_id': int(cls),
                    'confidence': float(conf),
                    'class_name': self.class_names[int(cls)],
                    'depth': depth_value,
                    'width_cm': obj_width_cm,
                    'height_cm': obj_height_cm
                }
                self.draw_detection(debug_image, detection_info)
            
            # Publish detections
            self.detections_pub.publish(detection_array)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in YOLO detection processing: {str(e)}")

    def draw_detection(self, image, detection):
        """Draw detection on image for visualization"""
        bbox = detection['bbox']
        center = detection['center']
        class_name = detection['class_name']
        confidence = detection['confidence']
        depth = detection['depth']
        width_cm = detection['width_cm']
        height_cm = detection['height_cm']
        
        # Color based on class ID
        colors = [
            (0, 255, 0),    # Green
            (255, 0, 0),    # Blue
            (0, 0, 255),    # Red
            (255, 255, 0),  # Cyan
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Yellow
        ]
        color = colors[detection['class_id'] % len(colors)]
        
        # Draw bounding box
        cv2.rectangle(image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        
        # Draw center point
        cv2.circle(image, center, 5, color, -1)
        
        # Prepare text
        text_lines = [f"{class_name}: {confidence:.2f}"]
        
        if depth is not None and depth > 0:
            depth_cm = depth / 10.0
            text_lines.append(f"Depth: {depth_cm:.1f} cm")
            
            if width_cm is not None and height_cm is not None:
                text_lines.append(f"Size: {width_cm:.1f}x{height_cm:.1f} cm")
        
        # Draw text background
        text_bg_height = len(text_lines) * 25 + 10
        cv2.rectangle(image, (bbox[0], bbox[1] - text_bg_height), 
                     (bbox[0] + 300, bbox[1]), color, -1)
        
        # Draw text
        for i, text in enumerate(text_lines):
            y_pos = bbox[1] - text_bg_height + 20 + i * 25
            cv2.putText(image, text, (bbox[0] + 5, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YoloDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
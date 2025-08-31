#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point, Pose2D
from cv_bridge import CvBridge
import cv2
import numpy as np
import os


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Declare parameters
        self.declare_parameter('detection_threshold', 0.7)
        self.declare_parameter('mask_threshold', 0.3)
        self.declare_parameter('model_path', 'dnn/frozen_inference_graph_coco.pb')
        self.declare_parameter('config_path', 'dnn/mask_rcnn_inception_v2_coco_2018_01_28.pbtxt')
        self.declare_parameter('classes_path', 'dnn/classes.txt')
        self.declare_parameter('use_cuda', False)
        
        # Get parameters
        self.detection_threshold = self.get_parameter('detection_threshold').get_parameter_value().double_value
        self.mask_threshold = self.get_parameter('mask_threshold').get_parameter_value().double_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self.classes_path = self.get_parameter('classes_path').get_parameter_value().string_value
        self.use_cuda = self.get_parameter('use_cuda').get_parameter_value().bool_value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize detection model
        self.setup_model()
        
        # Create subscribers
        self.color_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        
        # Create publishers
        self.detections_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.debug_image_pub = self.create_publisher(Image, '/debug/detected_objects', 10)
        
        # Store latest frames
        self.latest_color_image = None
        self.latest_depth_image = None
        
        self.get_logger().info('Object detection node started')

    def setup_model(self):
        """Initialize Mask R-CNN model"""
        try:
            # Check if model files exist
            if not os.path.exists(self.model_path):
                self.get_logger().warn(f"Model file not found: {self.model_path}")
                self.get_logger().warn("Using dummy detection for demonstration")
                self.use_dummy_detection = True
                return
            
            self.use_dummy_detection = False
            
            # Load the network
            self.net = cv2.dnn.readNetFromTensorflow(self.model_path, self.config_path)
            
            if self.use_cuda:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
                self.get_logger().info("Using CUDA acceleration")
            else:
                self.get_logger().info("Using CPU inference")
            
            # Generate random colors for visualization
            np.random.seed(2)
            self.colors = np.random.randint(0, 255, (90, 3))
            
            # Load class names
            self.classes = []
            if os.path.exists(self.classes_path):
                with open(self.classes_path, "r") as file_object:
                    for class_name in file_object.readlines():
                        class_name = class_name.strip()
                        self.classes.append(class_name)
            else:
                # Default COCO classes for demonstration
                self.classes = ['person', 'car', 'chair', 'bottle', 'cup']
                self.get_logger().warn(f"Classes file not found: {self.classes_path}, using default classes")
            
            self.get_logger().info(f"Loaded {len(self.classes)} object classes")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize detection model: {str(e)}")
            self.use_dummy_detection = True

    def color_callback(self, msg):
        """Handle color image messages"""
        try:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process detection if we have both color and depth
            if self.latest_depth_image is not None:
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
            if self.use_dummy_detection:
                detections = self.dummy_detection()
            else:
                detections = self.detect_objects_mask()
            
            # Create Detection2DArray message
            detection_array = Detection2DArray()
            detection_array.header = header
            
            debug_image = self.latest_color_image.copy()
            
            for detection in detections:
                # Create Detection2D message
                detection_msg = Detection2D()
                detection_msg.header = header
                
                # Set bounding box
                detection_msg.bbox.center = Point()
                detection_msg.bbox.center.x = float(detection['center'][0])
                detection_msg.bbox.center.y = float(detection['center'][1])
                detection_msg.bbox.size_x = float(detection['width'])
                detection_msg.bbox.size_y = float(detection['height'])
                
                # Set object hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(detection['class_id'])
                hypothesis.hypothesis.score = float(detection['confidence'])
                detection_msg.results.append(hypothesis)
                
                detection_array.detections.append(detection_msg)
                
                # Draw on debug image
                self.draw_detection(debug_image, detection)
            
            # Publish detections
            self.detections_pub.publish(detection_array)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in detection processing: {str(e)}")

    def detect_objects_mask(self):
        """Perform object detection using Mask R-CNN"""
        detections = []
        
        try:
            # Create blob from image
            blob = cv2.dnn.blobFromImage(self.latest_color_image, swapRB=True)
            self.net.setInput(blob)
            
            # Run forward pass
            boxes, masks = self.net.forward(["detection_out_final", "detection_masks"])
            
            frame_height, frame_width, _ = self.latest_color_image.shape
            detection_count = boxes.shape[2]
            
            for i in range(detection_count):
                box = boxes[0, 0, i]
                class_id = int(box[1])
                score = box[2]
                
                if score < self.detection_threshold:
                    continue
                
                # Extract bounding box coordinates
                x = int(box[3] * frame_width)
                y = int(box[4] * frame_height)
                x2 = int(box[5] * frame_width)
                y2 = int(box[6] * frame_height)
                
                # Calculate center and dimensions
                center_x = (x + x2) // 2
                center_y = (y + y2) // 2
                width = x2 - x
                height = y2 - y
                
                # Get depth information if available
                depth_value = None
                if self.latest_depth_image is not None:
                    depth_value = self.latest_depth_image[center_y, center_x]
                
                detection = {
                    'bbox': [x, y, x2, y2],
                    'center': (center_x, center_y),
                    'width': width,
                    'height': height,
                    'class_id': class_id,
                    'confidence': score,
                    'depth': depth_value,
                    'class_name': self.classes[class_id] if class_id < len(self.classes) else 'unknown'
                }
                
                detections.append(detection)
            
        except Exception as e:
            self.get_logger().error(f"Error in mask detection: {str(e)}")
        
        return detections

    def dummy_detection(self):
        """Dummy detection for testing when model files are not available"""
        detections = []
        
        if self.latest_color_image is not None:
            h, w = self.latest_color_image.shape[:2]
            
            # Create a dummy detection in the center of the image
            detection = {
                'bbox': [w//4, h//4, 3*w//4, 3*h//4],
                'center': (w//2, h//2),
                'width': w//2,
                'height': h//2,
                'class_id': 0,
                'confidence': 0.85,
                'depth': 1000 if self.latest_depth_image is not None else None,
                'class_name': 'dummy_object'
            }
            detections.append(detection)
        
        return detections

    def draw_detection(self, image, detection):
        """Draw detection on image for visualization"""
        bbox = detection['bbox']
        center = detection['center']
        class_name = detection['class_name']
        confidence = detection['confidence']
        depth = detection['depth']
        
        # Get color for this class
        color_idx = detection['class_id'] % len(self.colors)
        color = tuple(map(int, self.colors[color_idx]))
        
        # Draw bounding box
        cv2.rectangle(image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        
        # Draw center cross
        cv2.line(image, (center[0]-10, center[1]), (center[0]+10, center[1]), color, 2)
        cv2.line(image, (center[0], center[1]-10), (center[0], center[1]+10), color, 2)
        
        # Prepare text
        text_lines = [f"{class_name}: {confidence:.2f}"]
        if depth is not None:
            depth_cm = depth / 10.0  # Convert to cm
            text_lines.append(f"Depth: {depth_cm:.1f} cm")
        
        # Draw text background
        text_size = cv2.getTextSize(text_lines[0], cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        text_bg_height = len(text_lines) * 25 + 10
        cv2.rectangle(image, (bbox[0], bbox[1] - text_bg_height), 
                     (bbox[0] + max(250, text_size[0] + 10), bbox[1]), color, -1)
        
        # Draw text
        for i, text in enumerate(text_lines):
            y_pos = bbox[1] - text_bg_height + 20 + i * 25
            cv2.putText(image, text, (bbox[0] + 5, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ObjectDetectionNode()
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
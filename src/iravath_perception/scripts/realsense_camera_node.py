#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2


class RealsenseCameraNode(Node):
    def __init__(self):
        super().__init__('realsense_camera_node')
        
        # Declare parameters
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('enable_color', True)
        
        # Get parameters
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.enable_depth = self.get_parameter('enable_depth').get_parameter_value().bool_value
        self.enable_color = self.get_parameter('enable_color').get_parameter_value().bool_value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create publishers
        if self.enable_color:
            self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        if self.enable_depth:
            self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Initialize RealSense camera
        self.setup_camera()
        
        # Create timer for publishing frames
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)
        
        self.get_logger().info(f'RealSense camera node started - {self.width}x{self.height}@{self.fps}fps')

    def setup_camera(self):
        """Initialize RealSense camera pipeline"""
        try:
            self.get_logger().info("Loading Intel RealSense Camera")
            self.pipeline = rs.pipeline()
            
            config = rs.config()
            
            if self.enable_color:
                config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            if self.enable_depth:
                config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            
            # Start streaming
            profile = self.pipeline.start(config)
            
            # Create align object for aligning depth to color
            if self.enable_depth and self.enable_color:
                align_to = rs.stream.color
                self.align = rs.align(align_to)
            
            # Setup depth filtering
            if self.enable_depth:
                self.spatial_filter = rs.spatial_filter()
                self.spatial_filter.set_option(rs.option.holes_fill, 3)
                self.hole_filling_filter = rs.hole_filling_filter()
            
            # Get camera intrinsics for camera info
            if self.enable_color:
                self.color_stream = profile.get_stream(rs.stream.color)
                self.color_intrinsics = self.color_stream.as_video_stream_profile().get_intrinsics()
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize RealSense camera: {str(e)}")
            raise

    def create_camera_info_msg(self):
        """Create CameraInfo message"""
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        
        if hasattr(self, 'color_intrinsics'):
            msg.width = self.color_intrinsics.width
            msg.height = self.color_intrinsics.height
            msg.k = [
                self.color_intrinsics.fx, 0.0, self.color_intrinsics.ppx,
                0.0, self.color_intrinsics.fy, self.color_intrinsics.ppy,
                0.0, 0.0, 1.0
            ]
            msg.d = [self.color_intrinsics.coeffs[i] for i in range(5)]
            msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            msg.p = [
                self.color_intrinsics.fx, 0.0, self.color_intrinsics.ppx, 0.0,
                0.0, self.color_intrinsics.fy, self.color_intrinsics.ppy, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
        
        return msg

    def timer_callback(self):
        """Timer callback to capture and publish frames"""
        try:
            # Wait for frames
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            
            if self.enable_depth and self.enable_color:
                # Align depth to color if both are enabled
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
            else:
                depth_frame = frames.get_depth_frame() if self.enable_depth else None
                color_frame = frames.get_color_frame() if self.enable_color else None
            
            timestamp = self.get_clock().now().to_msg()
            
            # Process and publish color frame
            if self.enable_color and color_frame:
                color_image = np.asanyarray(color_frame.get_data())
                color_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
                color_msg.header.stamp = timestamp
                color_msg.header.frame_id = "camera_link"
                self.color_pub.publish(color_msg)
            
            # Process and publish depth frame
            if self.enable_depth and depth_frame:
                # Apply filters to depth frame
                filtered_depth = self.spatial_filter.process(depth_frame)
                filled_depth = self.hole_filling_filter.process(filtered_depth)
                
                depth_image = np.asanyarray(filled_depth.get_data())
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")
                depth_msg.header.stamp = timestamp
                depth_msg.header.frame_id = "camera_link"
                self.depth_pub.publish(depth_msg)
            
            # Publish camera info
            camera_info_msg = self.create_camera_info_msg()
            self.camera_info_pub.publish(camera_info_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error capturing frame: {str(e)}")

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        try:
            if hasattr(self, 'pipeline'):
                self.pipeline.stop()
                self.get_logger().info("RealSense camera stopped")
        except Exception as e:
            self.get_logger().error(f"Error stopping camera: {str(e)}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RealsenseCameraNode()
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
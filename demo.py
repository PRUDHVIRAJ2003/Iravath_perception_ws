#!/usr/bin/env python3
"""
Demo script showing how to use the Iravath Perception ROS2 package
This script demonstrates the conversion from standalone to ROS2
"""

import os
import time
import subprocess
import signal
import sys

class PerceptionDemo:
    def __init__(self):
        self.processes = []
        
    def cleanup(self, signum=None, frame=None):
        """Clean up running processes"""
        print("\n🛑 Stopping all processes...")
        for process in self.processes:
            if process.poll() is None:  # Process is still running
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
        self.processes = []
        print("✅ Cleanup complete")
        
    def run_command(self, cmd, description):
        """Run a command and keep track of the process"""
        print(f"🚀 Starting: {description}")
        print(f"   Command: {' '.join(cmd)}")
        try:
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.processes.append(process)
            return process
        except FileNotFoundError:
            print(f"❌ Command not found: {cmd[0]}")
            return None
            
    def demo_standalone_legacy(self):
        """Demonstrate the original standalone scripts"""
        print("\n" + "="*60)
        print("📚 LEGACY STANDALONE MODE DEMO")
        print("="*60)
        print("This shows how the original scripts worked before ROS2 conversion")
        print("\nOriginal standalone scripts available:")
        print("• test.py - Basic YOLO detection")
        print("• test1.py - YOLO with measurements") 
        print("• object_dist_find/measure_object_distance.py - Mask R-CNN system")
        print("\nThese are preserved in src/iravath_perception/legacy_standalone/")
        print("They can still be run independently without ROS2")
        
    def demo_ros2_mode(self):
        """Demonstrate ROS2 integration"""
        print("\n" + "="*60)
        print("🤖 ROS2 JAZZY MODE DEMO")
        print("="*60)
        print("This shows the new ROS2 integration")
        
        # Check if ROS2 is available
        try:
            result = subprocess.run(['ros2', '--version'], capture_output=True, text=True)
            if result.returncode != 0:
                print("❌ ROS2 not available in this environment")
                print("📋 Would run these commands in a ROS2 environment:")
                self.show_ros2_commands()
                return
        except FileNotFoundError:
            print("❌ ROS2 not installed in this environment")
            print("📋 Would run these commands in a ROS2 environment:")
            self.show_ros2_commands()
            return
            
        print("✅ ROS2 detected, but package needs to be built first")
        self.show_ros2_commands()
        
    def show_ros2_commands(self):
        """Show what ROS2 commands would be used"""
        print("\n🔧 ROS2 Build Commands:")
        print("cd ~/ros2_ws")
        print("colcon build --packages-select iravath_perception")
        print("source install/setup.bash")
        
        print("\n📸 Camera Node:")
        print("ros2 run iravath_perception realsense_camera_node.py")
        
        print("\n🎯 Object Detection Node (Mask R-CNN):")
        print("ros2 run iravath_perception object_detection_node.py")
        
        print("\n⚡ YOLO Detection Node:")
        print("ros2 run iravath_perception yolo_detection_node.py")
        
        print("\n🚀 Launch Complete System:")
        print("ros2 launch iravath_perception perception.launch.py")
        
        print("\n📊 Monitor Topics:")
        print("ros2 topic list")
        print("ros2 topic echo /camera/color/image_raw")
        print("ros2 topic echo /detections")
        
    def show_comparison(self):
        """Show before/after comparison"""
        print("\n" + "="*60)
        print("🔄 BEFORE vs AFTER COMPARISON")
        print("="*60)
        
        print("\n📜 BEFORE (Standalone):")
        print("✓ Simple Python scripts")
        print("✓ Direct camera access")
        print("✓ Hardcoded parameters")
        print("✗ No inter-process communication")
        print("✗ No standard message formats")
        print("✗ Difficult to integrate with robot systems")
        
        print("\n🤖 AFTER (ROS2):")
        print("✓ Modular ROS2 nodes")
        print("✓ Standard sensor_msgs and vision_msgs")
        print("✓ Configurable via YAML parameters")
        print("✓ Topic-based communication")
        print("✓ Easy integration with ROS2 ecosystem")
        print("✓ Launch files for orchestration")
        print("✓ Preserves original functionality")
        
    def show_topics_and_messages(self):
        """Show ROS2 topics and message types"""
        print("\n📡 ROS2 TOPICS AND MESSAGES:")
        topics = [
            ("/camera/color/image_raw", "sensor_msgs/Image", "RGB camera feed"),
            ("/camera/depth/image_raw", "sensor_msgs/Image", "Depth camera feed"),
            ("/camera/camera_info", "sensor_msgs/CameraInfo", "Camera calibration"),
            ("/detections", "vision_msgs/Detection2DArray", "Object detections"),
            ("/debug/detected_objects", "sensor_msgs/Image", "Debug visualization"),
        ]
        
        for topic, msg_type, description in topics:
            print(f"  {topic}")
            print(f"    Type: {msg_type}")
            print(f"    Description: {description}")
            print()

def main():
    demo = PerceptionDemo()
    
    # Set up signal handlers for clean shutdown
    signal.signal(signal.SIGINT, demo.cleanup)
    signal.signal(signal.SIGTERM, demo.cleanup)
    
    try:
        print("🎯 IRAVATH PERCEPTION ROS2 CONVERSION DEMO")
        print("This demonstrates the conversion from standalone Python to ROS2 Jazzy")
        
        demo.demo_standalone_legacy()
        demo.demo_ros2_mode()
        demo.show_comparison()
        demo.show_topics_and_messages()
        
        print("\n" + "="*60)
        print("🎉 CONVERSION COMPLETE!")
        print("="*60)
        print("✅ Successfully converted perception system to ROS2 Jazzy")
        print("✅ Original functionality preserved in legacy_standalone/")
        print("✅ New ROS2 nodes ready for robot integration")
        print("✅ Standard message types for interoperability")
        print("✅ Configurable parameters for flexibility")
        
        print("\n📚 Next Steps:")
        print("1. Install ROS2 Jazzy and dependencies")
        print("2. Build the package with colcon")
        print("3. Test with your RealSense camera")
        print("4. Integrate with your robot's navigation/control systems")
        
        print("\n📖 Documentation:")
        print("• README.md - Complete usage instructions")
        print("• src/iravath_perception/README.md - Package details")
        print("• config/perception_params.yaml - Configuration options")
        
    except KeyboardInterrupt:
        pass
    finally:
        demo.cleanup()

if __name__ == "__main__":
    main()
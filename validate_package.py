#!/usr/bin/env python3
"""
Validation script to check ROS2 package structure and dependencies
"""

import os
import sys
import subprocess
import importlib.util

def check_file_exists(filepath, description):
    """Check if a file exists"""
    if os.path.exists(filepath):
        print(f"✅ {description}: {filepath}")
        return True
    else:
        print(f"❌ {description}: {filepath} - NOT FOUND")
        return False

def check_python_import(module_name):
    """Check if a Python module can be imported"""
    try:
        spec = importlib.util.find_spec(module_name)
        if spec is not None:
            print(f"✅ Python module: {module_name}")
            return True
        else:
            print(f"❌ Python module: {module_name} - NOT FOUND")
            return False
    except ImportError:
        print(f"❌ Python module: {module_name} - IMPORT ERROR")
        return False

def main():
    print("🔍 Validating Iravath Perception ROS2 Package Structure")
    print("=" * 60)
    
    # Base paths
    workspace_root = "/home/runner/work/Iravath_perception_ws/Iravath_perception_ws"
    package_root = os.path.join(workspace_root, "src", "iravath_perception")
    
    success_count = 0
    total_checks = 0
    
    # Check package structure
    structure_checks = [
        (os.path.join(package_root, "package.xml"), "Package manifest"),
        (os.path.join(package_root, "CMakeLists.txt"), "CMake file"),
        (os.path.join(package_root, "setup.py"), "Setup file"),
        (os.path.join(package_root, "resource", "iravath_perception"), "Resource marker"),
        (os.path.join(package_root, "iravath_perception", "__init__.py"), "Python package init"),
        
        # Scripts
        (os.path.join(package_root, "scripts", "realsense_camera_node.py"), "Camera node script"),
        (os.path.join(package_root, "scripts", "object_detection_node.py"), "Object detection node"),
        (os.path.join(package_root, "scripts", "yolo_detection_node.py"), "YOLO detection node"),
        
        # Launch files
        (os.path.join(package_root, "launch", "perception.launch.py"), "Main launch file"),
        (os.path.join(package_root, "launch", "camera.launch.py"), "Camera launch file"),
        
        # Config
        (os.path.join(package_root, "config", "perception_params.yaml"), "Parameters file"),
        
        # Documentation
        (os.path.join(package_root, "README.md"), "Package README"),
        (os.path.join(workspace_root, "README.md"), "Workspace README"),
        
        # Legacy preservation
        (os.path.join(package_root, "legacy_standalone", "test.py"), "Legacy test script"),
        (os.path.join(package_root, "legacy_standalone", "object_dist_find"), "Legacy object detection"),
    ]
    
    print("\n📁 Package Structure:")
    for filepath, description in structure_checks:
        if check_file_exists(filepath, description):
            success_count += 1
        total_checks += 1
    
    # Check Python dependencies
    print("\n🐍 Python Dependencies:")
    python_deps = [
        "cv2",
        "numpy", 
        "torch",
        "pyrealsense2"
    ]
    
    for dep in python_deps:
        if check_python_import(dep):
            success_count += 1
        total_checks += 1
    
    # Check ROS2 dependencies (if available)
    print("\n🤖 ROS2 Dependencies:")
    ros_deps = [
        "rclpy",
        "sensor_msgs",
        "vision_msgs", 
        "cv_bridge",
        "geometry_msgs"
    ]
    
    for dep in ros_deps:
        if check_python_import(dep):
            success_count += 1
        total_checks += 1
    
    # Check executable permissions
    print("\n🔐 Executable Permissions:")
    executable_scripts = [
        os.path.join(package_root, "scripts", "realsense_camera_node.py"),
        os.path.join(package_root, "scripts", "object_detection_node.py"),
        os.path.join(package_root, "scripts", "yolo_detection_node.py"),
    ]
    
    for script in executable_scripts:
        if os.path.exists(script) and os.access(script, os.X_OK):
            print(f"✅ Executable: {os.path.basename(script)}")
            success_count += 1
        else:
            print(f"❌ Not executable: {os.path.basename(script)}")
        total_checks += 1
    
    # Summary
    print("\n" + "=" * 60)
    print(f"📊 Validation Summary: {success_count}/{total_checks} checks passed")
    
    if success_count == total_checks:
        print("🎉 All checks passed! Package is ready for ROS2 Jazzy.")
        return 0
    else:
        print(f"⚠️  {total_checks - success_count} issues found. Please review the failed checks above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
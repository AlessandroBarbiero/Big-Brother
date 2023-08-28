# Big-Brother
ROS2 Packages for Object Detection and Tracking with sensor fusion in autonomous vehicle scenario

## Dependencies
- ROS2 Foxy

### Yolo detector
Python packages:
- opencv-python
- cv bridge
- Ultralytics/YOLOv8 + [relative dependencies](https://github.com/ultralytics/ultralytics/blob/main/requirements.txt)

### Lidar detector
Following the instructions at [NVIDIA ROS2 TAO Pointpillars](https://github.com/NVIDIA-AI-IOT/ros2_tao_pointpillars/tree/main):

- TensorRT 8.2 (or above)
- TensorRT OSS 22.02
- [TAO Converter](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/tao/resources/tao-converter) 
# Big-Brother
ROS2 Packages for Object Detection and Tracking with sensor fusion in autonomous vehicle scenario

## Dependencies
- ROS2 Foxy

Download the necessary ros packages in the usual way
```cmd
sudo apt install ros-distro-package
```
for example:
```cmd
sudo apt install ros-foxy-vision-msgs
```

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

### Thermal detector
Python packages:
- opencv-python
- cv bridge

### Tracker


### Visualizer
Dependencies:
- [imgui](https://github.com/ocornut/imgui)
- [imgui-f](https://github.com/AlessandroBarbiero/ImGui-f) (A framework made by me to start easily an imgui based GUI)
- [implot](https://github.com/epezent/implot)
- [glfw](https://github.com/glfw/glfw)

Download the repositories and add the locations to the [CMakeLists](bb_utils/CMakeLists.txt) file

### Bag recording

```cmd
sudo apt install ros-foxy-rosbag2
```
To add a different storage option with a better compression:
```cmd
sudo apt install ros-foxy-rosbag2-storage-mcap
```

## Before run

Clone the repository in a ros2 workspace
```cmd
git clone https://github.com/AlessandroBarbiero/Big-Brother.git
```

Build first the `bb_interfaces` package
```cmd
colcon build --packages-select bb_interfaces
```

Build the packages with symlink-install
```cmd
colcon build --symlink-install
```

Source the overlay
```cmd
source /.../workspace/install/local_setup.sh
```
or
```cmd
source /.../workspace/install/local_setup.zsh
```

### Optional

Set the environmental variable BAG_DIR pointing at the directory where you store the bag files.

```cmd
export BAG_DIR=/path_to_directory/bag_files/
```

## How to use

### Detect & Track
Launch the Tracker with the detectors on a custom bag.
```cmd
ros2 launch bb_tracker detect_and_track.launch.py bag_name:=custom_bag
```

### Detect on bag
Launch the Detectors on a custom bag and register a new bag with detections. The bag registered in this way is slowed down, in order to make it easyer to register all the messages from a bag to another and give time to the different detectors.
```cmd
ros2 launch bb_detection detect_on_bag.launch.py bag_name:=custom_bag
```

To bring back the bag to the right speed use the `rewrite_bag_timestamps` executable from `bb_utils` package:
```cmd
ros2 run bb_utils rewrite_bag_timestamps /path/to/custom_bag /path/to/new_bag -a
```
# Big-Brother
ROS2 Packages for distributed multi-sensor Object Detection and Tracking in autonomous vehicle scenario.

This repository contains the code for the tracking algorithm developed in the paper  
'Heterogeneous Data Fusion for Accurate Road User Tracking: A Distributed Multi-Sensor Collaborative Approach'.

Everything is implemented within ROS2 Foxy to allow an easier integration in real scenarios offering packages for detection, tracking and testing.

<p align="center">
<img src="https://github.com/AlessandroBarbiero/Big-Brother/assets/79710064/5b56c790-c217-4819-973b-b7805f534598" alt="Custom map with 3D bounding boxes" width="600" />
</p>

## Abstract

This work presents the design and validation of
a distributed multi-sensor object tracking algorithm designed
to integrate heterogeneous sensory data from multiple static
acquisition stations. The primary challenge addressed is the
accurate tracking of targets in complex urban environments,
where occlusions and the dynamic nature of traffic frequently
hinder detection and tracking efforts. This challenge is particularly
relevant in multimodal exchange areas, where vehicular
traffic merges with heavy pedestrian and bicycle flow. We also
address the scenario of delayed detection, which can easily
occur when data from multiple stations are combined or when
intensive data processing is performed. Our algorithm ensures
high coverage and accuracy by maintaining dual Extended
Kalman Filter states for each object, thus allowing for the
assimilation of delayed detections and preserving optimal filter
estimates at all times. The results of the proposed pipeline,
tested using a digital twin of the Milano Bovisa Campus,
demonstrate its efficacy, achieving high tracking precision
across various scenarios and sensor combinations. Moreover,
the results highlight the advantages of a distributed multi-sensor
acquisition system compared to a single central station.

## Dependencies

Download the necessary ros packages in the usual way
```cmd
sudo apt install ros-distro-package
```
for example:
```cmd
sudo apt install ros-foxy-vision-msgs
```

In the following, a list of dependencies not related to ROS2 is given for each of the packages.

### BB Detection
Python packages:
- opencv-python
- numpy
- ultralytics

### BB Tracker
[Eigen3](https://eigen.tuxfamily.org/)

Download the repository and add the location to the [CMakeLists](bb_tracker/CMakeLists.txt) file.

### BB Utils

- [Dear ImGui](https://github.com/ocornut/imgui)
- [Dear ImGUI framework](https://github.com/AlessandroBarbiero/ImGui-f) (A framework to easily start an imgui based GUI)
- [ImPlot](https://github.com/epezent/implot)
- [GLFW](https://github.com/glfw/glfw)

Download the repositories and add the locations to the [CMakeLists](bb_utils/CMakeLists.txt) file

***

### Lidar detector
Within the repository is present also an attempt to integrate two different types of lidar detector.

If you want to download the dependencies for `Pointpillars` follow the instructions at [NVIDIA ROS2 TAO Pointpillars](https://github.com/NVIDIA-AI-IOT/ros2_tao_pointpillars/tree/main), the main dependencies are:

- TensorRT 8.2 (or above)
- TensorRT OSS 22.02
- [TAO Converter](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/tao/resources/tao-converter) 

If you want to work with `VISTA` you can build the docker container without further requirements.

### Scripts
[Scripts/requirements.txt](scripts/requirements.txt)

## Before run

Clone the repository in a ROS2 workspace
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

Set the environmental variable `BAG_DIR` pointing at the directory where you store the bag files. Some of the launch files use this variable to easily locate the bag files to run.  
There is no need to do this if you are not using the launch files to run the detectors or the tracker on a bag.

```cmd
export BAG_DIR=/path_to_directory/bag_files/
```

## How to use

### Track 
Simply launch the tracker passing the config file and open RViz to visualize the result.
```cmd
ros2 launch bb_tracker track.launch.py
```
If you want the tracker to operate on a bag with already registered detections type:
```cmd
ros2 launch bb_tracker track_on_bag.launch.py bag_name:=custom_bag
```

### Detect & Track
Launch the Tracker with the Detectors on a custom bag.
```cmd
ros2 launch bb_tracker detect_and_track.launch.py bag_name:=custom_bag
```
The launch file will search for a bag at the path specified within the environmental variable BAG_DIR.

### Detect on bag
Launch the Detectors on a custom bag and register a new bag with detections. The bag registered in this way is slowed down, in order to make it easier to register all the messages from a bag to another and give time to the different detectors to publish their messages.
```cmd
ros2 launch bb_detection detect_on_bag.launch.py bag_name:=custom_bag
```

To bring back the bag to the right speed use the `rewrite_bag_timestamps` executable from `bb_utils` package:
```cmd
ros2 run bb_utils rewrite_bag_timestamps /path/to/custom_bag /path/to/new_bag -a
```

### Evaluate the Tracker
Launch the tracker on a bag with registered detections and start the benchmark and visualizer node to study the behaviour of the tracker.
```cmd
ros2 launch bb_utils benchmark_tracking.launch.py bag_name:=custom_bag
```

## Results

In the following the results obtained in simulation.

| Metric | V1 | V2 | V3 |
|--------|----|----|----|
| TP     | 20157 | 343716 | 47559 |
| FP     | 315 | 45427 | 8218 |
| FN     | 89 | 6177 | 11098 |
| IDS    | 3852 | 65586 | 1953 |
| GT     | 20246 | 349893 | 58657 |
|-------|--------|--------|--------|
| DetA  | 0.9804 | 0.8695 | 0.7112 |
| LocA  | 0.8552 | 0.7840 | 0.7751 |
| AssA  | 0.8702 | 0.7668 | 0.7829 |
|-------|--------|--------|--------|
| DetRe | 0.9956 | 0.9823 | 0.8108 |
| DetPr | 0.9846 | 0.8833 | 0.8527 |
| DetF1 | 0.9901 | 0.9302 | 0.8312 |
| AssRe | 0.8081 | 0.7226 | 0.8494 |
| AssPr | 0.9965 | 0.9796 | 0.9919 |
| AssF1 | 0.8925 | 0.8317 | 0.9152 |
|-------|--------|--------|--------|
| MOTP  | 0.1448 | 0.2160 | 0.2249 |
| MOTA  |  0.7898| 0.6651 | 0.6374 |
|-------|--------|--------|--------|
| HOTA  | 0.8055 | 0.6617 | 0.5949 |

*Comparison of evaluation metrics for Multi-Object Tracking in different sensor scenarios. The values refer to tests performed in a simulation environment.*

V1 = Three sensor stations with only lidar (noisy ground truth).  
V2 = Three sensor stations with lidar, RGB and thermal (semantic segmentation) cameras.  
V3 = One sensor station with lidar, RGB and thermal (semantic segmentation) cameras.


## Acknowledgement

Part of the code takes inspiration from [ByteTrack](https://github.com/ifzhang/ByteTrack).
Many thanks for their inspiring works.

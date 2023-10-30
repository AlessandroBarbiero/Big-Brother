cmake_minimum_required(VERSION 3.5)
project(bb_tracker)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------ Add for Bytetrack
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

# Eigen3
include_directories(/usr/include/eigen3)  # <----- Point to an updated version of eigen3

# cuda
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARIES})

# ----------------------------

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(bb_interfaces REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)

# Add to include header files
include_directories(include)

# Add to run launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

file(GLOB BYTE_traker_files
    src/BYTEtracker.cpp 
    src/update2d.cpp
    src/update3d.cpp
    src/kalmanFilter.cpp 
    src/lapjv.cpp 
    src/STrack.cpp 
    src/utils.cpp
    src/EKF.cpp
)

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# EXECUTABLES
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

# %%%%%%% BB_TRACKER %%%%%%%%%%%
add_executable(bb_tracker 
  src/bb_tracker.cpp
  ${BYTE_traker_files}
  )
ament_target_dependencies(bb_tracker
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  vision_msgs
  visualization_msgs
  OpenCV
  cv_bridge
  image_geometry
  
  bb_interfaces
  )
# -----------

install(TARGETS 
  bb_tracker
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

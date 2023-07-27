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
include_directories(/usr/include/eigen3)

# include and link dirs of cuda and tensorrt
# cuda
find_package(CUDA REQUIRED)
# include_directories(/usr/local/cuda/include)
# link_directories(/usr/local/cuda/lib64)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES})

# cudnn
# include_directories(/data/cuda/cuda-10.2/cudnn/v8.0.4/include)
# link_directories(/data/cuda/cuda-10.2/cudnn/v8.0.4/lib64)
# tensorrt
# include_directories(/opt/tiger/demo/TensorRT-7.2.3.4/include)
# link_directories(/opt/tiger/demo/TensorRT-7.2.3.4/lib)
# cudnn and tensorrt
include_directories(/usr/include/x86_64-linux-gnu)
link_directories(/usr/lib/x86_64-linux-gnu)

# ----------------------------

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Add to include header files
include_directories(include)

# Add to run launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

file(GLOB BYTE_traker_files
    src/BYTEtracker.cpp 
    src/kalmanFilter.cpp 
    src/lapjv.cpp 
    src/STrack.cpp 
    src/utils.cpp
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
  # std_msgs
  vision_msgs
  OpenCV
  # bb_interfaces
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

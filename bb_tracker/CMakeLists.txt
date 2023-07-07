cmake_minimum_required(VERSION 3.5)
project(bb_tracker)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add for Bytetrack
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

include_directories(${PROJECT_SOURCE_DIR}include)
include_directories(/usr/include/eigen3)
link_directories(${PROJECT_SOURCE_DIR}/include)


# include and link dirs of cuda and tensorrt
# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# TODO add to use neural network
# # cudnn
# include_directories(/data/cuda/cuda-10.2/cudnn/v8.0.4/include)
# link_directories(/data/cuda/cuda-10.2/cudnn/v8.0.4/lib64)
# # tensorrt
# include_directories(/opt/tiger/demo/TensorRT-7.2.3.4/include)
# link_directories(/opt/tiger/demo/TensorRT-7.2.3.4/lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

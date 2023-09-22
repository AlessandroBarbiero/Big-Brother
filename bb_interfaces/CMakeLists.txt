cmake_minimum_required(VERSION 3.5)
project(bb_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vision_msgs REQUIRED)

set(msg_files
  "msg/Message.msg"
  "msg/Stats.msg"
)

set(srv_files
  "srv/ChangeMessage.srv"
  "srv/StopVisualizer.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES vision_msgs

  # DEPENDENCIES geometry_msgs # Add packages that above messages depend on
)

# Add this if you are exporting executable nodes in the same package
# ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

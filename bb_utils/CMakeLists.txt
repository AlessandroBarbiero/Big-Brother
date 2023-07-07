cmake_minimum_required(VERSION 3.5)
project(bb_utils)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(bb_interfaces REQUIRED) 
find_package(message_filters REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# Add to run launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# %%%%%%%%%%%%%%%%%%%%%%% TO USE IMGUI

find_package(Vulkan REQUIRED)

# GLFW
set(GLFW_DIR /opt/opengl/glfw-3.3.8) # -----> Set this to point to an up-to-date GLFW repo

option(GLFW_BUILD_EXAMPLES "Build the GLFW example programs" OFF)
option(GLFW_BUILD_TESTS "Build the GLFW test programs" OFF)
option(GLFW_BUILD_DOCS "Build the GLFW documentation" OFF)
option(GLFW_INSTALL "Generate installation target" OFF)
option(GLFW_DOCUMENT_INTERNALS "Include internals in documentation" OFF)
add_subdirectory(${GLFW_DIR} binary_dir EXCLUDE_FROM_ALL)
include_directories(${GLFW_DIR}/include)

# Dear ImGui
set(IMGUI_DIR /opt/imgui/)    # -------> Set this to point to an up-to-date Dear ImGui repo

include_directories(${IMGUI_DIR} ${IMGUI_DIR}/backends ..)

file(GLOB imgui_files
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp 
    ${IMGUI_DIR}/backends/imgui_impl_vulkan.cpp 
    ${IMGUI_DIR}/imgui.cpp 
    ${IMGUI_DIR}/imgui_draw.cpp 
    ${IMGUI_DIR}/imgui_demo.cpp 
    ${IMGUI_DIR}/imgui_tables.cpp 
    ${IMGUI_DIR}/imgui_widgets.cpp
)
# ImGui_f
include_directories(/opt/ImGui-f/src)

# Libraries
set(LIBRARIES "glfw;Vulkan::Vulkan")

# Use vulkan headers from glfw:
include_directories(${GLFW_DIR}/deps)

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# EXECUTABLES
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

# %%%%%%% VISUALIZER %%%%%%%%%%%
add_executable(bb_visualizer 
  src/bb_visualizer.cpp 
  ${imgui_files} # added imgui files
  )  
ament_target_dependencies(bb_visualizer 
  rclcpp
  std_msgs
  bb_interfaces
  )
# Added for ImGui
target_link_libraries(bb_visualizer
  ${LIBRARIES}
)



# -------------- other nodes
add_executable(bb_talker src/bb_publisher.cpp)
ament_target_dependencies(bb_talker
  rclcpp
  std_msgs
  vision_msgs
  # bb_interfaces
  )
add_executable(bb_listener src/bb_subscriber.cpp)
ament_target_dependencies(bb_listener 
  rclcpp
  std_msgs
  message_filters
  vision_msgs
  )
# ---------------

#--------------- BAG CONVERTER
add_executable(rewrite_bag_timestamps src/rewrite_bag_timestamps.cpp)
ament_target_dependencies(rewrite_bag_timestamps 
  rosbag2_cpp 
  rclcpp
  )
# ------------------

# target_include_directories(bb_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)


install(TARGETS 
  bb_visualizer
  bb_talker
  bb_listener
  rewrite_bag_timestamps
  DESTINATION lib/${PROJECT_NAME})



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

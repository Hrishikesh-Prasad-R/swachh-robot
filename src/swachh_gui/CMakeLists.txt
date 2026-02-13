cmake_minimum_required(VERSION 3.8)
project(swachh_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Find GTK4
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK4 REQUIRED gtk4)

# Main executable
add_executable(gui_teleop src/gui_teleop_node.cpp)

# ROS 2 dependencies
ament_target_dependencies(gui_teleop
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
)

# GTK4 includes and libraries
target_include_directories(gui_teleop PRIVATE ${GTK4_INCLUDE_DIRS})
target_link_directories(gui_teleop PRIVATE ${GTK4_LIBRARY_DIRS})
target_link_libraries(gui_teleop ${GTK4_LIBRARIES})
target_compile_options(gui_teleop PRIVATE ${GTK4_CFLAGS_OTHER} -Wall)

# Install
install(TARGETS gui_teleop DESTINATION lib/${PROJECT_NAME})

ament_package()

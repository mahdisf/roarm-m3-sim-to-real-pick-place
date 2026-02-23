cmake_minimum_required(VERSION 3.8)
project(roarm_p)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Build the C++ Node
add_executable(simple_pick_place src/simple_pick_place.cpp)
ament_target_dependencies(simple_pick_place
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  tf2
)

# Install the Node (The Executable)
install(TARGETS
  simple_pick_place
  DESTINATION lib/${PROJECT_NAME}
)

# --- CRITICAL STEP: Install the Launch File ---
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/visualize_path.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

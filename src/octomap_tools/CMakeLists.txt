cmake_minimum_required(VERSION 3.5)
project(octomap_tools)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

# Diretório com código C++
add_executable(save_octomap_bt src_cpp/save_octomap_bt.cpp)
ament_target_dependencies(save_octomap_bt rclcpp octomap_msgs octomap)

install(TARGETS
  save_octomap_bt
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


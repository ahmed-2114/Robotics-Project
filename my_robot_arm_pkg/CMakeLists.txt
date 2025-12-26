cmake_minimum_required(VERSION 3.8)
project(my_robot_arm_pkg)

# Dependencies required by Lab 3
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gripper_controllers REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_image REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(xacro REQUIRED)

# Install directories
install(
  DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
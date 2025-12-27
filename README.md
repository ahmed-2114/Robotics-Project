🤖 Custom 4-DOF Robotic Arm Project

Welcome to the Custom Robotic Arm Project repository! This project provides a complete ROS 2 pipeline for simulating, planning, and controlling a custom 4-degree-of-freedom robotic arm.

It seamlessly integrates Gazebo for physics simulation, MoveIt 2 for advanced motion planning, and a micro-ROS architecture to control physical hardware (ESP32) in real-time.

✨ Key Features

🦾 Full Simulation: Realistic physics and collision detection using Gazebo.

🧠 Motion Planning: Integrated MoveIt 2 configuration for trajectory planning and obstacle avoidance.

🔌 Sim-to-Real Bridge: A bridge node that formats simulation data for the microcontroller.

⚡ Micro-ROS Integration: Direct communication with ESP32 via the micro-ROS agent.

👁️ Visualization: Pre-configured RViz setups for monitoring joint states and planning paths.

🖥️ Terminal Dashboard (How to Run)

To run the full system, you will need 4 separate terminals. Follow this dashboard layout to get everything running.

Terminal 1: The Simulation

Terminal 2: The Brain

Terminal 3: The Bridge

Terminal 4: The Agent

Goal: Physics & Env

Goal: Planning & Control

Goal: Data Formatting

Goal: Hardware Link

ros2 launch my_robot_arm_pkg gazebo.launch.py

ros2 launch my_robot_arm_moveit demo.launch.py

python3 src/bridge_node.py

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

✅ Gazebo Window Opens



✅ Robot Spawns

✅ RViz Window Opens



✅ Interactive Markers

✅ Subscribes /joint_states



✅ Publishes Command

✅ Connects to ESP32



✅ Syncs Topics

📂 Project Structure

The repository is organized into main components:

Directory

Description

my_robot_arm_pkg

Core package containing the robot description (URDF/Xacro), meshes (.STL), and Gazebo simulation launch files.

my_robot_arm_moveit

Generated MoveIt 2 configuration package. Contains SRDF, kinematics configs, and motion planning launch files.

src/bridge_node.py

A utility node that converts JointState messages into Float32MultiArray for the ESP32.

micro_ros_publisher.ino

The ESP32 firmware code that receives ROS 2 commands and drives the servos.

🛠️ Prerequisites

Before you begin, ensure you have the following installed:

OS: Ubuntu 22.04 (Jammy)

ROS 2: Humble Hawksbill

Packages: gazebo_ros_pkgs, ros2_control, moveit, ros_gz

Micro-ROS: Micro-ROS Agent installed on your host machine.

🚀 Installation Guide

1. Create a Workspace

mkdir -p ~/robot_ws/src
cd ~/robot_ws/src


2. Clone the Repository

Clone this repo into your src folder:

git clone <your-repo-url-here> .


3. Install Dependencies

cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y


4. Build the Project

colcon build
source install/setup.bash


🔌 Hardware & Firmware Setup (ESP32)

This project uses micro-ROS to communicate with the ESP32. The firmware file is included as micro_ros_publisher.ino.

1. Wiring & Pinout

Connect your servos to the ESP32 using the following pin mapping defined in the firmware:

Joint Name

Servo Variable

ESP32 GPIO Pin

Joint 1

servo_j1

GPIO 13

Joint 2

servo_j2

GPIO 12

Joint 3

servo_j3

GPIO 14

Joint 4

servo_j4

GPIO 27

Gripper

servo_gripper

GPIO 26

2. Flashing the ESP32

Open micro_ros_publisher.ino in the Arduino IDE.

Install the required libraries:

micro_ros_arduino

ESP32Servo

Compile and upload to your ESP32 board.

Note: The code includes logic to constrain servos between 0-180 degrees and handles an offset calculation for Joint 2.

3. Run the Agent

Connect your ESP32 via USB and run the agent to expose the microcontroller to the ROS 2 network:

# Replace /dev/ttyUSB0 with your actual port
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200


📦 Main Dependencies

robot_state_publisher

joint_state_publisher

rviz2

xacro

ros_gz_sim

ros_gz_bridge

controller_manager

micro_ros_agent

👤 Maintainers

Name: Ahmed

Email: ahmed.shabaan2114@gmail.com

Built by the Autonomous Team at Ain Shams University.

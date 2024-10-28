NXP Cup - Pixy and LiDAR Based Autonomous Car Simulation

This repository contains the code and files for an autonomous car simulation developed for the NXP Cup. The car utilizes Pixy and LiDAR sensors to follow a track, avoid obstacles, and respond to various track signals.
Table of Contents

    Project Overview
    Features
    Project Structure
    Setup Instructions
    Usage
    Contributing

Project Overview

The project combines ROS2 for communication and control, Python for scripting the main behavior, and Pixy and LiDAR data to achieve robust autonomous driving and obstacle avoidance.
Features

    Autonomous Mode: Tracks lines using edge vectors.
    Obstacle Detection: Uses LIDAR for real-time detection.
    Traffic Control: Responds to stop signs and turns.

Project Structure

    cranium - Core files for processing Pixy camera and LIDAR data.
    electrode, helmet - Modules for integrating other sensors.
    installer, node_modules - Setup and dependency files.
    b3rb_ros_line_follower.py - Main code for line-following and obstacle detection.

Setup Instructions

    Clone the repository:

    bash

git clone https://github.com/VyomVyas25/NXP_Cup-Pixy_and_LiDAR_based_autonomous_car_simulation.git

Navigate to the project directory and install dependencies:

bash

cd NXP_Cup-Pixy_and_LiDAR_based_autonomous_car_simulation
./installer/setup.sh

Launch the simulation environment:

bash

    ros2 launch b3rb_gz_bringup sil.launch.py world:=Raceway_1

Usage

    Run the Line-Following Node:

    bash

ros2 run b3rb_ros_line_follower vectors

Start the Vehicle Controller:

bash

    ros2 run b3rb_ros_line_follower runner

Contributing

Please feel free to contribute to this project by submitting issues or pull requests.


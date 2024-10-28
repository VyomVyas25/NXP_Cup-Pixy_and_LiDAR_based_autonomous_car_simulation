# NXP_Cup-Pixy_and_LiDAR_based_autonomous_car_simulation

NXP Cup - Pixy and LiDAR Based Autonomous Car Simulation


## Overview

    -This project is part of the NXP Cup, where a simulated autonomous car is programmed to follow a track, detect obstacles, and      respond to various visual and sensor inputs using Pixy and LiDAR data. This setup combines ROS2 for communication, Python for control algorithms, and specific messages for line detection, obstacle avoidance, and speed adjustments.


## Project Structure

    1. cranium - Core files for processing Pixy camera data and LIDAR input.
    2. electrode, helmet - Modules for sensor integration.
    3. installer, node_modules - Setup and dependency files for running the project.
    4. b3rb_ros_line_follower.py - Main Python file controlling the vehicle, including line following and obstacle detection.


## Setup Instructions

    Clone the Repository
      git clone https://github.com/VyomVyas25/NXP_Cup-Pixy_and_LiDAR_based_autonomous_car_simulation.git
    Install Dependencies
      cd NXP_Cup-Pixy_and_LiDAR_based_autonomous_car_simulation
      ./installer/setup.sh
    Run the Simulation
      ros2 launch b3rb_gz_bringup sil.launch.py world:=Raceway_1
    Run the nodes
      ros2 run b3rb_ros_line_follower vectors
      ros2 run b3rb_ros_line_follower runner

      
## Key Features

    1.Autonomous Mode: Line following using edge vectors.
    2.Obstacle Detection: LIDAR scans for real-time obstacle avoidance.
    3.Traffic Control: Responds to stop signs and turns based on traffic conditions.

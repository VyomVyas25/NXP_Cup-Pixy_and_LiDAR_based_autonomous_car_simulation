# NXP Cup - Pixy and LiDAR Based Autonomous Car Simulation

This repository contains code for a simulated autonomous car built for the NXP Cup. The car is programmed to follow a line, detect obstacles, and respond to track conditions using data from Pixy and LiDAR sensors.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Project Structure](#project-structure)
- [Setup Instructions](#setup-instructions)
- [Usage](#usage)
- [Contributing](#contributing)

## Overview
This project uses **ROS2** for inter-component communication and control, along with Python scripts and sensor data (from Pixy and LiDAR) for autonomous driving, line-following, and obstacle detection.

## Features
1. **Autonomous Mode** - Tracks lines using edge vectors.
2. **Obstacle Detection** - Uses LiDAR for real-time obstacle detection.
3. **Traffic Control** - Responds to stop signs and track turns.

## Project Structure
1. **`cranium`** - Contains core files for processing Pixy camera and LiDAR data.
2. **`electrode`, `helmet`** - Modules for integrating additional sensors.
3. **`installer`, `node_modules`** - Setup and dependency files for running the project.
4. **`b3rb_ros_line_follower.py`** - Main Python file for line following and obstacle detection.
5. **`final_autonomous.py` and `edge_vectors.py`** - These files implement the core autonomous logic and line-following algorithms. Note that other files in the repository are included for installation or supplementary purposes.

## Setup Instructions

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/VyomVyas25/NXP_Cup-Pixy_and_LiDAR_based_autonomous_car_simulation.git
2. **Install Dependencies**:
   ```bash
   cd NXP_Cup-Pixy_and_LiDAR_based_autonomous_car_simulation./installer/setup.sh
3. **Launch the Simulation**:
   ```bash
   ros2 launch b3rb_gz_bringup sil.launch.py world:=Raceway_1
4. **Run Line Following**:
   ```bash
   ros2 run b3rb_ros_line_follower vectors
5. **Start the Vehicle Controller**:
   ```bash
   ros2 run b3rb_ros_line_follower runner

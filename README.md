# NXP_Cup-Pixy_and_LiDAR_based_autonomous_car_simulation

NXP Cup - Pixy and LiDAR Based Autonomous Car Simulation
Overview
    This project is part of the NXP Cup, where a simulated autonomous car is programmed to follow a track, detect obstacles, and      respond to various visual and sensor inputs using Pixy and LiDAR data. This setup combines ROS2 for communication, Python for     control algorithms, and specific messages for line detection, obstacle avoidance, and speed adjustments.

Project Structure
    cranium - Core files for processing Pixy camera data and LIDAR input.
    electrode, helmet - Modules for sensor integration.
    installer, node_modules - Setup and dependency files for running the project.
    b3rb_ros_line_follower.py - Main Python file controlling the vehicle, including line following and obstacle detection.

Setup Instructions
   1. Clone the Repository
      git clone https://github.com/VyomVyas25/NXP_Cup-Pixy_and_LiDAR_based_autonomous_car_simulation.git
   2. Install Dependencies
      cd NXP_Cup-Pixy_and_LiDAR_based_autonomous_car_simulation
      ./installer/setup.sh
   3. Run the Simulation
      ros2 launch b3rb_gz_bringup sil.launch.py world:=Raceway_1
   4. Run the nodes
      ros2 run b3rb_ros_line_follower vectors
      ros2 run b3rb_ros_line_follower runner
      
Key Features
    Autonomous Mode: Line following using edge vectors.
    Obstacle Detection: LIDAR scans for real-time obstacle avoidance.
    Traffic Control: Responds to stop signs and turns based on traffic conditions.

# Self Moving Car: AI/ML 🏎️🤖

An autonomous vehicle project developed by a team of 9, 4 from AI/ML Department, 2 from Electronic Department, and 3 from Mechanical Department. This project follows a phased approach, starting from basic actuation to full autonomous navigation using ROS 2 and Computer Vision.


# 📁 Repository Structure

    docs/: Project documentation, circuit diagrams, and phase reports.

    firmware/: Codes for the Arduino Mega (Motor control and sensor drivers).

    ros2_ws/: The ROS 2 Humble workspace containing the high-level intelligence and simulation environments.


# 🛠️ Project Phases

    Phase	    Objective	                                        Status
    Phase 0 	Basic Actuation (Manual U-Turns/Movement)	        ✅ Completed
    Phase 1 	Data Perception & Simulation (Gazebo/ROS 2)	🏗️      In Progress
    Phase 2 	Autonomy & Algorithm Integration (SLAM/YOLO)	    📅 Planned
    Phase 3 	Hardware Deployment & Real-World Testing	        📅 Planned


# 👥 Cross-Functional Team

Computer Science & AI/ML Department

    Kunal Singh (ROS Lead): System Architecture, Simulation (Gazebo), Middleware.

    Yash Dutt (OpenCV Lead): Computer Vision & Lane Detection.

    Abhay (Sensor Fusion Lead): Odometry & EKF Integration.

    Shuksham (SLAM Lead): Mapping & Navigation Stack.

Electronics & Communication Department

    Chirag: Sensor Interfacing, Logic Level Shifting

    Varun Kundnani: Motor driver calibration, BMS, Power Management

Mechanical Engineering Department

    Prashast Jain: Chassis Design, Material Selection, Drivetrain Assembly

    Anant Sidana: Chassis Design, Material Selection, Weight Distribution

    Deepanshu: Chassis Design, Wheel Alignment


# 🚀 Phase 1: Simulation Goals

We are currently in Phase 1, focusing on a Simulation-First approach using ROS 2 Humble and Gazebo Fortress.

    Develop a URDF model (Digital Twin) of the chassis.

    Setup a Virtual Environment with lanes and obstacles.

    Perform Data Logging (Rosbags) for AI training.

    Establish a Serial Bridge between ROS 2 and Arduino Mega.


# ⚙️ Requirements

    OS: Ubuntu 22.04 LTS

    Middleware: ROS 2 Humble

    Simulator: Gazebo Fortress

    Hardware: Raspberry Pi 4, Jetson Nano, Arduino Mega, RPLidar A1, MPU6050, etc.
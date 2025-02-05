# Aruco Marker Detection and Robot Control

## Overview
This program detects Aruco markers in the environment using a camera, processes them to identify and sort the markers by their IDs, and controls the robot's rotation and camera to align with the detected markers. The program uses ROS for communication, OpenCV for image processing, and the Aruco library for marker detection.

## Features

### Marker Detection
- Detects Aruco markers from the camera feed.
- Draws a bounding box around detected markers.
- Sorts markers based on their IDs and assigns a position for circular order.

### Image Publishing
- Publishes the image with detected markers drawn on it to the `result` topic.

### Robot Rotation
- Rotates the robot to align with the detected markers in a predefined order.
- Calculates the optimal rotation direction (clockwise or counterclockwise) based on the markers' positions.

### Dynamic Camera Control
- Publishes velocity commands to adjust the robot's camera position dynamically.

## Dependencies
- **ROS** (Robot Operating System)
- **OpenCV** (for image processing)
- **aruco** (for marker detection)
- **cv_bridge** (for converting ROS image messages to OpenCV format)
- **sensor_msgs**, **geometry_msgs**, and **std_msgs** (ROS message types)

## Topics Used

### Subscribed Topics
- `/robot/camera1/image_raw`: Input image stream from the robot's camera.

### Published Topics
- `result`: Image stream with detected markers annotated.
- `/cmd_vel`: Velocity commands to control robot movement.
- `/robot/camera_joint_velocity_controller/command`: Velocity commands to control the camera's movement.

## Nodes Overview

### 1. `marker_publish.cpp`
- **Path**: `aruco_ros/aruco_ros/src/marker_publish.cpp`
- **Description**: This node handles the functionality for the first task, where the entire robot rotates to scan and detect ArUco markers in the environment. The rotation allows the robot to cover its surroundings and identify markers effectively.

### 2. `marker_publish2.cpp`
- **Path**: `aruco_ros/aruco_ros/src/marker_publish2.cpp`
- **Description**: This node is designed for the second task, where only the robot's camera rotates to scan and detect ArUco markers. This approach enables marker detection while keeping the robot stationary.


## How to Launch the File

This program supports two tasks, each with its own launch file for specific functionality:

### Task 1: Rotate the Robot and Find Markers
In this task, the robot rotates in place to detect and align with markers placed in a circular arrangement in the environment. To launch this task, execute the following command:

```bash
roslaunch robot_urdf sim_aruco.launch
```
### Task 2:  Rotate the Camera and Find Markers
In this task, only the robot's camera rotates to detect markers. Unlike the first task, the camera may not be able to detect markers positioned behind the robot's wheels due to limited visibility, for that I tried to change the world to make the camera able to capture all the markers . To launch this task, execute the following command:
```bash
roslaunch robot_urdf sim_aruco2.launch

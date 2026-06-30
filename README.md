# Autonomous-Warehouse-Robot
An autonomous warehouse robot simulation built with ROS2 Humble, Gazebo, and TurtleBot3, demonstrating LiDAR-based obstacle avoidance, A* path planning, and object recognition for automated pick-and-place operations.


📋 Project Overview

This project simulates a warehouse robot capable of:


Navigating a virtual warehouse autonomously
Avoiding obstacles in real time using LiDAR sensor data
Planning optimal paths between warehouse waypoints using the A* algorithm
Recognizing objects (warehouse items) for automated picking
Executing a complete pick-and-place mission using a finite state machine



🛠️ Tools & Technologies

ComponentTechnologyRobotics FrameworkROS2 HumbleSimulationGazeboRobot PlatformTurtleBot3 (Burger)LanguagePython 3.10Sensors2D LiDAR, CameraDevelopment PlatformThe Construct Sim


🏗️ System Architecture

┌──────────────────┐     /goal_pose      ┌───────────────────────┐
│  A* Planner       │────────────────────▶│                       │
└──────────────────┘                      │                       │
                                           │  Mission Controller   │────▶ /cmd_vel ────▶ TurtleBot3
┌──────────────────┐  /detected_object    │  (Finite State        │
│  Object Detector  │────────────────────▶│   Machine)            │
└──────────────────┘                      │                       │
                                           └───────────────────────┘
┌──────────────────┐
│  LiDAR Avoider    │────────────────────▶ /cmd_vel (obstacle override)
└──────────────────┘
        ▲
        │ /scan
        │
   TurtleBot3 LiDAR Sensor


🔄 Mission State Machine

   ┌──────┐
   │ IDLE │
   └──┬───┘
      │
      ▼
┌─────────────┐      ┌───────────┐      ┌─────────┐      ┌──────────┐      ┌───────────┐
│ NAVIGATING  │─────▶│ DETECTING │─────▶│ PICKING │─────▶│ PLACING  │─────▶│ RETURNING │
└─────────────┘      └───────────┘      └─────────┘      └──────────┘      └───────────┘
      ▲                                                          │                 │
      └──────────────────────(repeat for next item)──────────────┘                 │
                                                                                     ▼
                                                                                  IDLE


📦 Package Structure

warehouse_robot/
├── warehouse_robot/
│   ├── __init__.py
│   ├── astar_planner.py        # A* path planning node
│   ├── lidar_avoider.py        # LiDAR obstacle avoidance node
│   ├── object_detector.py      # Object recognition node
│   └── mission_controller.py   # FSM mission controller node
├── resource/
│   └── warehouse_robot
├── docs/
│   └── demo.mp4                # Demo video
├── package.xml
├── setup.py
└── README.md


⚙️ Node Descriptions

1. astar_planner.py

Implements the A* search algorithm to compute optimal paths between warehouse waypoints. Uses Euclidean distance as the heuristic function, evaluates 8-directional grid neighbors, and publishes computed goals to /goal_pose.

2. lidar_avoider.py

Subscribes to /scan (LiDAR data) and divides the scan into front, left, and right sectors. If an obstacle is detected within a 0.35m safety threshold, the robot automatically steers away by publishing corrective velocity commands to /cmd_vel.

3. object_detector.py

Performs HSV color-space thresholding to detect and classify warehouse items (boxes/crates) for pick-and-place. Publishes detection results with item type and location to /detected_object.

4. mission_controller.py

The core Finite State Machine (FSM) that orchestrates the full mission: navigates to a shelf, detects an item, executes a pick action, places it at the drop zone, and returns home — repeating until all items are delivered.

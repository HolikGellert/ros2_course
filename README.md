# ROS 2 Wall Follower with PD Control & Search

## Overview
This ROS 2 package implements an advanced wall-following algorithm for the TurtleBot3. It uses a PD (Proportional-Derivative) controller for smooth navigation and includes a state machine to handle corners and open spaces.

## Key Features
- **PD Controller:** Eliminates oscillation ("snaking") by reacting to the rate of error change (D-term).
- **Smart Corner Handling:**
  - **Inner Corners:** Detects frontal obstacles and turns right to avoid collision.
  - **Outer Corners:** Detects sudden wall endings and wraps around the corner (turns left).
- **Search Mode:** If the robot is placed in an open space (no wall nearby), it moves straight to find a wall instead of spinning in circles.
- **Dynamic Speed:** Moves faster on straight paths and slows down for precise corrections.

## Prerequisites
- **ROS 2 Distribution:** Humble / Foxy
- **OS:** Ubuntu 22.04 (Humble) or 20.04 (Foxy)

## Simulation Setup (Gazebo)

Before using this package, you must install the Gazebo simulator and the TurtleBot3 simulation packages.

1. **Install Gazebo and ROS 2 bindings:**
   *(Replace `humble` with `foxy` if you are using ROS 2 Foxy)*

   ```bash
   sudo apt update
   sudo apt install ros-humble-gazebo-ros-pkgs
   ```

2. **Install TurtleBot3 Packages:**

   ```bash
   sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-simulations
   sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-teleop
   ```

3. **Set the Robot Model:**
   Add the robot model to your bash configuration so you don't have to export it every time:

   ```bash
   echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
   source ~/.bashrc
   ```

## Installation

### 1. Clone this repository into your workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/HolikGellert/ros2_course.git
```

### 2. Build the package:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

### 1. Launch Simulation

Start Gazebo with your world. If you configured the launch file in this package:

```bash
ros2 launch ros2_course simulation.launch.py
```

*Alternatively, launch the standard empty world:*
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 2. Run the Node

Start the wall follower logic:

```bash
ros2 run ros2_course wall_follower
```

## Control Logic (State Machine)

The robot decides its movement based on LIDAR data using the following priority:

* **Emergency (Front < 0.45m):** Obstacle detected ahead. **Action:** Stop and spot turn RIGHT.
* **Search (Left > 2.0m):** No wall detected nearby. **Action:** Drive STRAIGHT fast to find a wall.
* **Outer Corner (1.2m < Left < 2.0m):** The wall ended recently. **Action:** Turn LEFT to follow the corner.
* **Wall Following (Left < 1.2m):** Wall detected. **Action:** Apply PD Control to maintain a 0.5m distance.

## Configuration

Parameters can be adjusted in `wall_follower.py`:

* `self.target_dist`: Desired distance from wall (0.5m).
* `self.kp`: Proportional gain (1.5).
* `self.kd`: Derivative gain (10.0).
* `self.max_wall_dist`: Threshold for switching to Search Mode (2.0m).

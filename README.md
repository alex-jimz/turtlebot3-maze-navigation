# TurtleBot3 Maze Navigation

This project was developed for the Robotics course at **FIB-UPC** and implements a two-phase autonomous navigation pipeline for **TurtleBot3 Burger** using ROS C++. You can watch the robot explore and complete the maze in `maze_video.mp4`.

## Project overview

The project turns LiDAR and odometry streams into action in real time so a robot can leave a maze reliably and, once mapped, navigate more directly using a planned route.

- **Phase 1 (exploration + mapping)**: `src/maze_12f.cpp` follows a right-hand wall-following strategy. It detects walls and junctions from laser scan sectors, estimates pose from odometry, builds a graph of junctions, and stores the map as a YAML file.
- **Phase 2 (navigation on map)**: `src/navigator_node.cpp` loads the saved graph, computes a shortest path with BFS from junction `0` to the farthest discovered junction, and executes that route while still checking local wall/odometry feedback.

The graph map is saved at:

- `/tmp/maze_graph.yaml`

## Repository contents

- `src/maze_12f.cpp`: maze exploration and graph construction node.
- `src/navigator_node.cpp`: route planner/executor node.
- `launch/maze_12f.launch`: launches the exploration node.
- `launch/navigator_node.launch`: launches the navigator node.
- `launch/explorer_node.launch`: launch entry for `explorer_node_exe` (kept for compatibility).
- `package.xml`: ROS package metadata.
- `CMakeLists.txt`: build targets and links (`roscpp`, `geometry_msgs`, `yaml-cpp`).

## Prerequisites

- ROS (catkin-based distribution, e.g. Noetic/Melodic/Kinetic).
- TurtleBot3 Burger environment providing:
  - `odom` (`nav_msgs/Odometry`)
  - `scan` (`sensor_msgs/LaserScan`)
  - `cmd_vel` publisher accepted (`geometry_msgs/Twist`)
- `yaml-cpp` installed.

## Build

Run these commands from the root of your catkin workspace:

```bash
cd <catkin_ws>/src
git clone <this_repo>
cd <catkin_ws>
catkin_make
source devel/setup.bash
```

## Run (two-phase use)

1. **Map the maze (exploration phase):**

```bash
roslaunch maze_12f maze_12f.launch
```

Let the robot explore until it reaches the exit. The process logs each discovered node/edge and writes `/tmp/maze_graph.yaml`.

2. **Navigate using the map (guided phase):**

```bash
roslaunch maze_12f navigator_node.launch
```

This launches the navigator that reads `/tmp/maze_graph.yaml`, computes the shortest route, and drives to the exit using the planned directions.

## Notes

- Launch files are plain ROS launch files (no custom command line arguments in this version).
- If you use `explorer_node.launch`, ensure a valid `src/explorer_node.cpp` target exists for that launch target before building, or use `maze_12f.launch` for exploration.

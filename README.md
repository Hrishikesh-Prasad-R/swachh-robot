# Swachh Robot Simulation

An autonomous robot simulation using ROS 2 Humble, Gazebo, and TurtleBot3 that navigates and covers a room while avoiding obstacles.

## Features

- 🤖 **Autonomous Navigation**: Robot explores environment without human input
- 🚧 **Obstacle Avoidance**: Uses laser scan to detect and avoid obstacles
- 🔄 **State Machine**: FORWARD and TURNING states with intelligent transitions
- 🎲 **Random Exploration**: Randomness prevents repetitive patterns

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- TurtleBot3 packages

## Quick Start

### 1. Build the workspace

```bash
cd ~/Desktop/swachh-robot-simulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Set environment variables

```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

### 3. Launch the simulation

```bash
ros2 launch swachh_robot simulation.launch.py
```

This single command will:
- Launch Gazebo with a simple room world
- Spawn the TurtleBot3 robot
- Start the autonomous navigation node

## Manual Commands

### Start Gazebo separately
```bash
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
    /opt/ros/humble/share/turtlebot3_gazebo/worlds/empty_world.world &
gzclient &
```

### Spawn robot manually
```bash
ros2 run gazebo_ros spawn_entity.py -entity turtlebot3_burger \
    -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
    -x 0.0 -y 0.0 -z 0.01
```

### Run autonomous navigator
```bash
ros2 run swachh_robot autonomous_navigator
```

### Monitor topics
```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /cmd_vel
```

## Parameters

The autonomous navigator supports these parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `obstacle_distance` | 0.5 | Distance (m) to trigger obstacle avoidance |
| `forward_speed` | 0.2 | Forward velocity (m/s) |
| `turn_speed` | 0.5 | Angular velocity (rad/s) |
| `turn_duration` | 2.0 | Time to turn when avoiding obstacle (s) |

## Project Structure

```
swachh-robot-simulation/
├── src/
│   └── swachh_robot/
│       ├── launch/
│       │   └── simulation.launch.py    # Main launch file
│       ├── worlds/
│       │   └── simple_room.world       # 6x6m room with obstacles
│       └── swachh_robot/
│           └── autonomous_navigator.py # Navigation node
├── install/                            # Built packages
└── README.md
```

## Troubleshooting

### Gazebo spawn service timeout
If robot spawn fails, increase the delay or spawn manually after Gazebo fully loads:
```bash
# Wait for Gazebo, then:
ros2 service list | grep spawn  # Should show /spawn_entity
ros2 run gazebo_ros spawn_entity.py -entity turtlebot3_burger -file ...
```

### Robot not moving
Check if autonomous_navigator is running:
```bash
ros2 node list | grep autonomous
ros2 topic echo /cmd_vel  # Should show velocity commands
```

## Next Steps

- [ ] Add SLAM for mapping
- [ ] Implement coverage path planning
- [ ] Add wall-following behavior
- [ ] Move to real hardware

## License

MIT

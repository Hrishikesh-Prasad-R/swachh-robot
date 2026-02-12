# Autonomous Vacuum Robot Simulation

A ROS 2 Humble simulation of an autonomous vacuum robot using TurtleBot3 and Ignition Gazebo.

## Prerequisites

- Ubuntu 22.04 (ARM64 or x86_64)
- ROS 2 Humble
- Ignition Gazebo (gz-sim)
- TurtleBot3 packages

## Quick Start

```bash
# Terminal 1: Build and launch the simulation
cd /home/orinova/Desktop/swacch/hmi/simulation
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
colcon build
source install/setup.bash
ros2 launch vacuum_bot vacuum_sim.launch.py
```

## Manual Launch (Alternative)

```bash
# Terminal 1: Launch Gazebo with robot
cd /home/orinova/Desktop/swacch/hmi/simulation
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch spawn_robot.launch.py

# Terminal 2: Run autonomous node
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vacuum_bot autonomous_vacuum
```

## Expected Behavior

- Robot spawns in a 10m x 10m enclosed room with walls and obstacles
- Moves forward autonomously at 0.2 m/s
- Detects obstacles via laser scan at 0.5m threshold
- Turns randomly (1-2.5 seconds) when obstacle detected
- Continuously explores room

## Monitor Topics

```bash
# List all topics
ros2 topic list

# Echo laser scan data
ros2 topic echo /scan

# Echo velocity commands
ros2 topic echo /cmd_vel
```

## Parameters

Edit `autonomous_vacuum.py` to adjust:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `obstacle_threshold` | 0.5m | Detection distance |
| `forward_speed` | 0.2 m/s | Movement speed |
| `turn_speed` | 0.8 rad/s | Rotation speed |
| `turn_duration` | 1.0-2.5s | Random turn duration |

## Troubleshooting

```bash
# Kill all Gazebo processes
killall -9 ign gazebo gzserver gzclient

# Rebuild from clean
cd /home/orinova/Desktop/swacch/hmi/simulation
rm -rf build install log
colcon build

# Check node status
ros2 node list
ros2 topic hz /scan
```

## File Structure

```
simulation/
├── worlds/
│   └── simple_room.world      # Gazebo world file
├── launch/
│   └── spawn_robot.launch.py  # Robot spawn launch file
├── src/
│   └── vacuum_bot/            # ROS 2 package
│       ├── vacuum_bot/
│       │   └── autonomous_vacuum.py
│       ├── launch/
│       │   └── vacuum_sim.launch.py
│       ├── package.xml
│       └── setup.py
└── README.md
```

# 08 — API / ROS Topic Interaction Map

## Complete ROS Topic Map

```mermaid
graph LR
    subgraph Publishers["Publishers"]
        GZ_SCAN["Gazebo LiDAR Plugin"]
        GZ_ODOM["Gazebo DiffDrive Plugin"]
        GZ_IMU["Gazebo IMU Plugin"]
        GZ_CAM["Gazebo Camera Plugin"]
        GZ_CLK["Gazebo ROS Init"]
        AT_PUB["ArrowTeleop"]
        AN_PUB["AutonomousNavigator"]
        SR_PUB["SlamReadiness"]
        EV_PUB["EnvironmentViz"]
        BM_PUB["BoxMarkerPublisher"]
        RSP["robot_state_publisher"]
    end

    subgraph Topics["ROS 2 Topics"]
        scan["/scan"]
        odom["/odom"]
        imu["/imu"]
        cmd_vel["/cmd_vel"]
        cam_rgb["/camera/image_raw"]
        cam_depth["/camera/depth/image_raw"]
        cam_info["/camera/camera_info"]
        cam_pts["/camera/points"]
        clock["/clock"]
        env_markers["/environment_markers"]
        robot_marker["/robot_marker"]
        tf["/tf"]
        tf_static["/tf_static"]
        robot_desc["/robot_description"]
    end

    subgraph Subscribers["Subscribers"]
        AT_SUB["ArrowTeleop"]
        AN_SUB["AutonomousNavigator"]
        SV_SUB["SensorValidator"]
        SR_SUB["SlamReadiness"]
        BM_SUB["BoxMarkerPublisher"]
        RVIZ["RViz2"]
    end

    GZ_SCAN --> scan
    GZ_ODOM --> odom
    GZ_IMU --> imu
    GZ_CAM --> cam_rgb
    GZ_CAM --> cam_depth
    GZ_CAM --> cam_info
    GZ_CAM --> cam_pts
    GZ_CLK --> clock
    RSP --> tf
    RSP --> tf_static
    RSP --> robot_desc

    AT_PUB --> cmd_vel
    AN_PUB --> cmd_vel
    SR_PUB --> cmd_vel
    EV_PUB --> env_markers
    BM_PUB --> robot_marker

    scan --> AT_SUB
    scan --> AN_SUB
    scan --> SV_SUB
    scan --> SR_SUB
    odom --> SV_SUB
    odom --> SR_SUB
    odom --> BM_SUB
    env_markers --> RVIZ
    robot_marker --> RVIZ
    cam_rgb --> RVIZ
    tf --> RVIZ
```

## Topic Registry

| Topic | Message Type | Publisher | Subscriber(s) | Rate |
|-------|-------------|----------|---------------|------|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo LiDAR | ArrowTeleop, AutoNav, SensorVal, SlamReady | 5 Hz |
| `/odom` | `nav_msgs/Odometry` | Gazebo DiffDrive | SensorVal, SlamReady, BoxMarker | ~30 Hz |
| `/imu` | `sensor_msgs/Imu` | Gazebo IMU | (unused — future ORB-SLAM3) | 200 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | ArrowTeleop / AutoNav / SlamReady | Gazebo DiffDrive | Variable |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo Camera | (future ORB-SLAM3) | 15 fps |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Gazebo Camera | (future ORB-SLAM3) | 15 fps |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo Camera | (future ORB-SLAM3) | 15 fps |
| `/camera/points` | `sensor_msgs/PointCloud2` | Gazebo Camera | (future ORB-SLAM3) | 15 fps |
| `/environment_markers` | `visualization_msgs/MarkerArray` | EnvironmentViz | RViz2 | 1 Hz |
| `/robot_marker` | `visualization_msgs/Marker` | BoxMarker | RViz2 | ~30 Hz |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo | All nodes (sim time) | ~1000 Hz |
| `/tf` | `tf2_msgs/TFMessage` | robot_state_publisher, Gazebo | SensorVal, SlamReady, RViz | Variable |
| `/tf_static` | `tf2_msgs/TFMessage` | robot_state_publisher | RViz | Latched |
| `/robot_description` | `std_msgs/String` | robot_state_publisher | RViz | Latched |

## Topic Conflict Analysis

> [!WARNING]
> **`/cmd_vel` contention**: Three nodes can publish to `/cmd_vel` — `ArrowTeleop`, `AutonomousNavigator`, and `SlamReadiness`. The `start_simulation.sh` script mitigates this by only launching one at a time, but the launch file (`simulation.launch.py`) starts autonomous_navigator alongside teleop, which would cause conflicting commands.

## HMI Serial Protocol

| Direction | Format | Example |
|-----------|--------|---------|
| **HMI → Arduino** | `COMMAND:PARAM\n` | `VACUUM:ON\n` |
| **Arduino → HMI (ACK)** | `ACK:RESULT\n` | `ACK:VACUUM_ON\n` |
| **Arduino → HMI (Status)** | `STATUS:K=V,K=V,...\n` | `STATUS:VAC=1,ARM=0,...\n` |
| **Arduino → HMI (Error)** | `ERR:MESSAGE\n` | `ERR:EMERGENCY_STOP_ACTIVE\n` |

### Command Reference

| Command | Response | Effect |
|---------|----------|--------|
| `VACUUM:ON` | `ACK:VACUUM_ON` | Activate vacuum relay |
| `VACUUM:OFF` | `ACK:VACUUM_OFF` | Deactivate vacuum relay |
| `ARM:ON/OFF` | `ACK:ARM_ON/OFF` | Control robotic arm |
| `WIPER:ON/OFF` | `ACK:WIPER_ON/OFF` | Control wiper motor |
| `UV:ON/OFF` | `ACK:UV_ON/OFF` | Control UV strip |
| `MOVE:FORWARD` | `ACK:MOVE_FORWARD` | Drive forward (manual only) |
| `MOVE:BACKWARD` | `ACK:MOVE_BACKWARD` | Drive backward (manual only) |
| `MOVE:LEFT` | `ACK:TURN_LEFT` | Turn left (manual only) |
| `MOVE:RIGHT` | `ACK:TURN_RIGHT` | Turn right (manual only) |
| `MOVE:STOP` | `ACK:MOTORS_STOPPED` | Stop all motors |
| `AUTO:ON/OFF` | `ACK:AUTO_MODE_ON/OFF` | Toggle autonomous mode |
| `ESTOP` | `ACK:EMERGENCY_STOP` | Emergency stop all systems |
| `RESET` | `ACK:SYSTEM_RESET` | Clear emergency stop |

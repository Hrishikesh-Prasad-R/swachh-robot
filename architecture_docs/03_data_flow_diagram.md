# 03 — Data Flow Diagram

## Level 0 — Context Diagram

```mermaid
graph LR
    User((User)) -- "Keyboard Input" --> System["Swachh Robot<br/>Simulation System"]
    System -- "3D Visualization" --> User
    System -- "Camera Feed" --> User
    System -- "Log Files" --> Disk[(Filesystem)]
    Gazebo["Gazebo Physics<br/>Engine"] <-- "Sensor Data<br/>Motor Commands" --> System
```

## Level 1 — Subsystem View

```mermaid
graph TB
    subgraph Input["Input Sources"]
        KB["Keyboard<br/>(Arrow Keys)"]
        LiDAR["Simulated LiDAR<br/>(360° @ 5Hz)"]
        Camera["Simulated RGB-D<br/>Camera (15fps)"]
        Odom["Odometry<br/>(Wheel Encoders)"]
        IMU["IMU Sensor<br/>(200Hz)"]
    end

    subgraph Processing["Processing Nodes"]
        AT["ArrowTeleop"]
        AN["AutonomousNavigator"]
        SV["SensorValidator"]
        SR["SlamReadiness"]
        EV["EnvironmentViz"]
        BM["BoxMarkerPublisher"]
    end

    subgraph Output["Outputs"]
        CmdVel["/cmd_vel<br/>(Twist commands)"]
        Markers["/environment_markers<br/>/robot_marker"]
        Logs["Log Files<br/>(logs/*.log)"]
        Report["SLAM Ready<br/>Report (stdout)"]
    end

    subgraph Simulation["Gazebo Simulation"]
        Physics["Physics Engine"]
        Rendering["Rendering Engine"]
    end

    KB --> AT
    LiDAR -- "/scan" --> AT
    LiDAR -- "/scan" --> AN
    LiDAR -- "/scan" --> SV
    LiDAR -- "/scan" --> SR
    Odom -- "/odom" --> SV
    Odom -- "/odom" --> SR
    Odom -- "/odom" --> BM
    Camera -- "/camera/rgb/image_raw" --> Output

    AT --> CmdVel
    AN --> CmdVel
    SR --> CmdVel
    CmdVel --> Physics
    Physics --> LiDAR
    Physics --> Odom
    Physics --> Camera
    Physics --> IMU

    EV --> Markers
    BM --> Markers
    SV --> Logs
    AT --> Logs
    SR --> Logs
    SR --> Report
```

## Level 2 — ArrowTeleop Internal Data Flow

This is the most complex node — it processes raw key input, LiDAR data, and produces velocity commands with collision avoidance.

```mermaid
graph TB
    subgraph Input
        Keys["stdin<br/>(raw terminal)"]
        Scan["/scan<br/>(LaserScan 360°)"]
    end

    subgraph Processing["ArrowTeleop Processing"]
        ReadKey["read_key()<br/>Non-blocking stdin"]
        ScanCB["scan_callback()<br/>Filter vision/collision cones"]
        MedianFilter["Median Filter<br/>(deque maxlen=3)"]
        SpeedFactor["get_speed_factor()<br/>Linear ramp: gap/SLOW_ZONE"]
        EffSpeed["get_effective_forward_speed()<br/>base_speed × factor"]
        ProcessKey["process_key()<br/>Map key → action"]
        CollisionEnf["Continuous Collision<br/>Enforcement (every tick)"]
    end

    subgraph Output
        CmdVel["/cmd_vel (Twist)"]
        LogOut["AsyncLogger"]
    end

    Keys --> ReadKey
    ReadKey --> ProcessKey
    Scan --> ScanCB
    ScanCB -- "collision_dist<br/>(120° cone)" --> MedianFilter
    ScanCB -- "vision_dist<br/>(60° cone)" --> MedianFilter
    MedianFilter --> SpeedFactor
    SpeedFactor --> EffSpeed
    EffSpeed --> ProcessKey
    ProcessKey --> CmdVel
    CollisionEnf --> CmdVel
    ProcessKey --> LogOut
    CollisionEnf --> LogOut
```

## Data Transformation Details

| Stage | Input | Transformation | Output |
|-------|-------|---------------|--------|
| **LiDAR → Collision Distance** | 360 range readings | Filter to 120° front cone, take min | Single float (meters) |
| **Collision Distance → Speed Factor** | Float distance | Linear ramp: `gap / SLOW_ZONE` clamped [0, 1] | Float [0.0–1.0] |
| **Speed Factor → Effective Speed** | Factor + base speed | `speed_levels[current] × factor` | Float (m/s) |
| **Key + Speed → Twist** | Keycode + effective speed | Map to linear.x / angular.z | Twist message |
| **Scan → Sectors** | 360 readings (autonomous) | Divide into N sectors, min per sector | N floats |
| **Sectors → Turn Direction** | N sector distances | Compare left-half avg vs right-half avg | ±1 integer |

## Sync vs Async Flows

| Flow | Type | Details |
|------|------|---------|
| `/scan` → `scan_callback` | **Async** (event-driven) | ROS subscriber callback on DDS message arrival |
| Keyboard → `process_key` | **Sync** (polling) | `read_key()` polls stdin every loop iteration |
| Collision enforcement | **Sync** (continuous) | Runs every iteration of the main `while` loop |
| `report()` checks | **Async** (timer-driven) | ROS timer fires every 5s (SensorValidator) |
| Async logging | **Async** (non-blocking) | Queued writes to file, never blocks caller |
| HMI → Arduino | **Async** (serial event) | GTK button clicks trigger serial writes; Arduino processes in `loop()` |

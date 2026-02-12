# 05 — Sequence Diagrams

## 1. Primary Workflow: Simulation Startup

```mermaid
sequenceDiagram
    participant User
    participant Script as start_simulation.sh
    participant GzServer as gzserver
    participant GzClient as gzclient
    participant Spawn as spawn_entity.py
    participant RSP as robot_state_publisher
    participant RViz as rviz2
    participant Teleop as arrow_teleop
    participant EnvViz as environment_viz
    participant SV as sensor_validator

    User->>Script: bash start_simulation.sh
    Script->>Script: Kill stale processes (pkill)
    Script->>Script: Source ROS 2 + workspace
    Script->>Script: Set TURTLEBOT3_MODEL=burger
    Script->>GzServer: Launch gzserver with simple_room.world
    Script->>Script: Wait for /clock topic
    Script->>GzClient: Launch gzclient (GUI)
    Script->>RSP: Launch robot_state_publisher
    Script->>Spawn: Spawn TurtleBot3 (camera model)
    Spawn->>GzServer: Insert model SDF
    GzServer-->>Spawn: Entity spawned
    Script->>RViz: Launch rviz2 with config
    Script->>EnvViz: Launch environment_viz
    Script->>SV: Launch sensor_validator
    Script->>Teleop: Launch arrow_teleop (foreground)
    Teleop->>Teleop: Set terminal to raw mode
    Teleop-->>User: Ready for keyboard input
```

## 2. Teleop Control Flow (Key Press → Robot Movement)

```mermaid
sequenceDiagram
    participant User
    participant Teleop as ArrowTeleop
    participant DDS as ROS 2 DDS
    participant Gazebo as Gazebo Physics

    loop Every main loop iteration
        User->>Teleop: Press ↑ (UP arrow)
        Teleop->>Teleop: read_key() → '\x1b[A'
        Teleop->>Teleop: get_effective_forward_speed()
        Note over Teleop: collision_dist=2.5m > SLOW_ZONE<br/>factor=1.0, speed=0.22 m/s
        Teleop->>DDS: publish /cmd_vel (linear.x=0.22)
        DDS->>Gazebo: Deliver Twist message
        Gazebo->>Gazebo: Apply forces to wheels
        Gazebo->>DDS: Publish /scan (360 ranges)
        DDS->>Teleop: scan_callback(LaserScan)
        Teleop->>Teleop: Filter 120° collision cone
        Teleop->>Teleop: Median filter → collision_dist
    end

    Note over User,Gazebo: When wall approaches...
    Gazebo->>DDS: /scan shows obstacle at 0.4m
    DDS->>Teleop: scan_callback()
    Teleop->>Teleop: collision_dist=0.4m
    Teleop->>Teleop: gap=0.4-0.105=0.295m<br/>factor=0.295/0.5=0.59
    Teleop->>DDS: publish /cmd_vel (linear.x=0.13)
    Note over Teleop: Proportional deceleration

    Gazebo->>DDS: /scan shows obstacle at 0.1m
    DDS->>Teleop: scan_callback()
    Teleop->>Teleop: gap ≤ 0 → factor=0.0
    Teleop->>DDS: publish /cmd_vel (linear.x=0.0)
    Note over Teleop: BLOCKED — hard stop
```

## 3. SLAM Readiness Check Flow

```mermaid
sequenceDiagram
    participant User
    participant SR as SlamReadiness
    participant DDS as ROS 2 DDS
    participant Gazebo
    participant TF as TF2 Buffer

    User->>SR: ros2 run swachh_robot slam_readiness
    SR->>SR: Start 15s timer
    SR-->>User: "Running checks for 15s..."

    loop t=0 to t=15s
        Gazebo->>DDS: /scan, /odom
        DDS->>SR: scan_cb(), odom_cb()
        SR->>SR: Accumulate data
    end

    Note over SR: t=3s: Send test movement
    SR->>DDS: /cmd_vel (linear.x=0.1)
    Note over SR: t=5s: Stop test
    SR->>DDS: /cmd_vel (linear.x=0.0)

    Note over SR: t=15s: Run all 7 checks
    SR->>SR: Check 1: Scan rate ~5Hz?
    SR->>SR: Check 2: NaN < 5%?
    SR->>TF: Check 3: TF chain complete?
    TF-->>SR: odom→base_footprint ✓
    SR->>SR: Check 4: Odom > 5Hz?
    SR->>SR: Check 5: Robot moved > 0.01m?
    SR->>SR: Check 6: Wall detected < 5m?
    SR->>SR: Check 7: Max gap < 500ms?

    SR-->>User: SLAM READINESS REPORT<br/>✅ ALL CHECKS PASSED
    SR->>SR: raise SystemExit(0)
```

## 4. Autonomous Navigation State Transitions

```mermaid
sequenceDiagram
    participant AN as AutonomousNavigator
    participant DDS as ROS 2 DDS
    participant Gazebo

    Note over AN: State: FORWARD
    loop Control loop (10Hz)
        Gazebo->>DDS: /scan
        DDS->>AN: scan_callback()
        AN->>AN: Calculate min_distance
        AN->>AN: Divide into sectors

        alt min_distance ≥ obstacle_distance
            AN->>DDS: /cmd_vel (linear.x=0.2)
            Note over AN: Continue FORWARD
        else min_distance < obstacle_distance
            AN->>AN: Compare left vs right sectors
            AN->>AN: Choose turn direction
            AN->>AN: 20% random flip
            Note over AN: Transition → TURNING
            AN->>DDS: /cmd_vel (angular.z=±0.5)
        end
    end

    Note over AN: State: TURNING
    loop Until turn_duration elapsed
        AN->>DDS: /cmd_vel (angular.z=±0.5)
    end

    alt Path clear after turn
        Note over AN: Transition → FORWARD
    else Still blocked
        AN->>AN: 30% chance flip direction
        Note over AN: Continue TURNING
    end
```

## 5. HMI → Arduino Command Flow

```mermaid
sequenceDiagram
    participant User
    participant HMI as GTK3 HMI
    participant Serial as Serial Port
    participant Arduino

    User->>HMI: Click "Vacuum ON" button
    HMI->>HMI: on_vacuum_toggle()
    HMI->>Serial: write("VACUUM:ON\n")
    Serial->>Arduino: "VACUUM:ON\n"
    Arduino->>Arduino: processCommand("VACUUM:ON")
    Arduino->>Arduino: digitalWrite(VACUUM_RELAY, HIGH)
    Arduino->>Serial: "ACK:VACUUM_ON\n"
    Serial->>HMI: Read response
    HMI->>HMI: Update status label

    Note over User,Arduino: Every 1 second...
    Arduino->>Serial: "STATUS:VAC=1,ARM=0,..."
    Serial->>HMI: Read status
    HMI->>HMI: Update all indicators

    User->>HMI: Click "Emergency Stop"
    HMI->>Serial: write("ESTOP\n")
    Arduino->>Arduino: emergencyStop()
    Arduino->>Arduino: Stop all motors + subsystems
    Arduino->>Serial: "ACK:EMERGENCY_STOP\n"
```

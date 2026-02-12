# 01 — System Architecture

## Architecture Type

**Distributed Publish-Subscribe** — The system follows ROS 2's decoupled node architecture where independent processes communicate via topic-based message passing. Each node is a separate OS process.

## Layered Architecture Diagram

```mermaid
graph TB
    subgraph Presentation["Presentation Layer"]
        RViz["RViz2 — 3D Visualization"]
        GazeboGUI["Gazebo Client — Physics Sim GUI"]
        Terminal["Terminal — Raw Key Input"]
        HMI_GUI["GTK3 HMI — Hardware Control GUI"]
    end

    subgraph Application["Application / Control Layer (ROS 2 Nodes)"]
        Teleop["ArrowTeleop<br/>arrow_teleop.py"]
        AutoNav["AutonomousNavigator<br/>autonomous_navigator.py"]
        SlamReady["SlamReadiness<br/>slam_readiness.py"]
    end

    subgraph Domain["Domain / Service Layer (ROS 2 Nodes)"]
        EnvViz["EnvironmentViz<br/>environment_viz.py"]
        BoxMarker["BoxMarkerPublisher<br/>box_marker.py"]
        SensorVal["SensorValidator<br/>sensor_validator.py"]
    end

    subgraph Infrastructure["Infrastructure Layer"]
        AsyncLog["AsyncLogger<br/>async_logger.py"]
        GazeboSrv["Gazebo Server — Physics Engine"]
        ROS2DDS["ROS 2 DDS Middleware"]
    end

    subgraph Hardware["Hardware Layer (Separate Subsystem)"]
        Arduino["Arduino Firmware<br/>swacch.ino"]
    end

    subgraph Data["Data Layer"]
        LogFiles["logs/ — Async Log Files"]
        WorldFile["simple_room.world — Environment Definition"]
        ModelSDF["model.sdf — Robot Definition"]
    end

    Terminal --> Teleop
    Teleop --> ROS2DDS
    AutoNav --> ROS2DDS
    SlamReady --> ROS2DDS
    EnvViz --> ROS2DDS
    BoxMarker --> ROS2DDS
    SensorVal --> ROS2DDS
    ROS2DDS --> GazeboSrv
    ROS2DDS --> RViz
    GazeboSrv --> GazeboGUI
    Teleop --> AsyncLog
    SensorVal --> AsyncLog
    SlamReady --> AsyncLog
    AsyncLog --> LogFiles
    GazeboSrv --> WorldFile
    GazeboSrv --> ModelSDF
    HMI_GUI -- "Serial 115200" --> Arduino
```

## Technology Stack

| Layer | Technology |
|-------|-----------|
| **OS** | Ubuntu 22.04 LTS |
| **Middleware** | ROS 2 Humble Hawksbill (DDS) |
| **Simulator** | Gazebo Classic 11 |
| **Visualization** | RViz2 |
| **Language (Sim)** | Python 3.10 |
| **Language (HMI)** | C++ (GTK3) |
| **Language (FW)** | Arduino C++ |
| **Build System** | Colcon (ament_python) |
| **Robot Platform** | TurtleBot3 Burger (simulated) |
| **Sensors** | 2D LiDAR (360°), IMU, RGB-D Camera |

## Deployment Boundaries

```mermaid
graph LR
    subgraph PC["Development PC (Ubuntu 22.04)"]
        subgraph ROS2["ROS 2 Workspace"]
            Nodes["6 Python Nodes"]
            Launch["Launch System"]
        end
        subgraph Gazebo["Gazebo Simulation"]
            Server["gzserver"]
            Client["gzclient"]
        end
        RViz2["RViz2"]
    end

    subgraph Docker["Docker (Future — ORB-SLAM3)"]
        ORBSLAM["ORB-SLAM3 Wrapper"]
    end

    subgraph HW["Physical Hardware (Separate)"]
        GTK["HMI Application"]
        MCU["Arduino Mega/Uno"]
        Motors["BLDC Motors"]
        Vacuum["Vacuum System"]
    end

    ROS2 <--> Gazebo
    ROS2 <--> RViz2
    ROS2 -.-> Docker
    GTK -- "Serial USB" --> MCU
    MCU --> Motors
    MCU --> Vacuum
```

> **Key Insight**: The simulation and hardware subsystems are **fully decoupled**. There is no ROS bridge between the GTK HMI/Arduino and the simulation nodes. They are designed to eventually converge when the physical robot is built.

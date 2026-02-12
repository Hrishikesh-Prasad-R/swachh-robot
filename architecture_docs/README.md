# Swachh Robot — System Architecture Documentation

## Table of Contents

| # | Document | Description |
|---|----------|-------------|
| 1 | [System Architecture](01_system_architecture.md) | High-level layered architecture, tech stack |
| 2 | [Component Diagram](02_component_diagram.md) | Deep dive into every module/node |
| 3 | [Data Flow Diagram](03_data_flow_diagram.md) | DFD Level 0, 1, 2 — how data moves |
| 4 | [Use Case Diagram](04_use_case_diagram.md) | Actors, use cases, entry points |
| 5 | [Sequence Diagrams](05_sequence_diagrams.md) | Key workflow step-by-step flows |
| 6 | [State Model](06_state_model.md) | State machines for navigation & system |
| 7 | [Data Model](07_data_model.md) | ROS message types, frame tree, entities |
| 8 | [API Interaction Map](08_api_interaction_map.md) | All ROS topics, services, TF frames |
| 9 | [Deployment & Runtime](09_deployment_runtime.md) | How to run, dependencies, config |
| 10 | [Risks & Observations](10_risks_observations.md) | Tech debt, bottlenecks, improvements |

## System Overview

The Swachh Robot project is a **ROS 2 Humble** simulation of an autonomous vacuum cleaner robot, built on the **TurtleBot3 Burger** platform in **Gazebo Classic 11**. It consists of two independent subsystems:

1. **Simulation Subsystem** — ROS 2 nodes for teleoperation, autonomous navigation, sensor validation, and environment visualization
2. **Hardware HMI Subsystem** — GTK3 desktop application + Arduino firmware for controlling a physical vacuum cleaner robot

These subsystems are **currently decoupled** — the simulation does not communicate with the HMI/Arduino hardware.

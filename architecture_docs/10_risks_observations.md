# 10 — Risks, Bottlenecks, and Design Observations

## Critical Issues

### 1. `/cmd_vel` Topic Contention

> [!CAUTION]
> Three nodes publish to `/cmd_vel`: `ArrowTeleop`, `AutonomousNavigator`, and `SlamReadiness`. ROS 2 provides no built-in arbitration — the last publisher wins.

**Evidence**: `arrow_teleop.py` L:53 publishes to `/cmd_vel`, `autonomous_navigator.py` L:52 publishes to the same topic, and `slam_readiness.py` L:63 also publishes.

**Risk**: If the launch file (`simulation.launch.py` L:72-88) starts `autonomous_navigator` alongside `arrow_teleop`, both send conflicting Twist commands, causing erratic robot behavior.

**Mitigation**: The shell script only launches one at a time, but this is fragile. Consider implementing a `cmd_vel_mux` multiplexer or using namespaced topics.

---

### 2. Hardcoded Physical Constants

> [!WARNING]
> Robot dimensions, collision thresholds, and speed levels are hardcoded as module-level constants, not ROS parameters.

**Files affected**:
- `arrow_teleop.py`: `ROBOT_RADIUS = 0.105`, `SLOW_ZONE = 0.5`, `speed_levels = [0.1, 0.22, 0.5]`
- `environment_viz.py`: All wall positions hardcoded to match `simple_room.world`

**Risk**: Changing the robot model or world layout requires editing multiple files manually. Values may silently diverge.

**Recommendation**: Move to ROS parameters or a shared config file.

---

### 3. No Topic Namespacing

> [!WARNING]
> All topics use global names (`/scan`, `/cmd_vel`, `/odom`). No namespace prefixing is used.

**Risk**: Impossible to run multiple robot instances. Any second robot would collide on all topics.

**Recommendation**: Use `<namespace>` in the robot model or `push_ros_namespace` in the launch file.

---

## Tight Coupling Areas

| Coupling | Description | Severity |
|----------|-------------|----------|
| **environment_viz ↔ simple_room.world** | Wall positions are hardcoded in Python to match the SDF file exactly. Any wall change requires manual sync in both files. | **High** |
| **arrow_teleop ↔ terminal** | Uses raw `termios` for keyboard input, tightly coupled to Linux TTY. Cannot run in a container or headless environment. | **Medium** |
| **start_simulation.sh ↔ model path** | Absolute path `/home/bmscecse/Desktop/swachh-robot-simulation/...` hardcoded in script. | **High** |
| **sensor_validator ↔ expected rates** | Expected scan rate (5Hz) and odom rate (30Hz) are hardcoded. Changing Gazebo sensor rates will trigger false warnings. | **Low** |

---

## Scalability Concerns

| Concern | Impact | Mitigation |
|---------|--------|-----------|
| **Software rendering** (`LIBGL_ALWAYS_SOFTWARE=1`) | Gazebo performance limited by CPU rendering. Camera at 640×480@15fps may drop frames. | Requires GPU for production use |
| **Single-threaded nodes** | All nodes use `rclpy.spin()` (single-threaded executor). Heavy scan processing could delay callbacks. | Use `MultiThreadedExecutor` for SensorValidator |
| **No QoS configuration** | All subscribers use default QoS (reliable, keep last 10). No `BEST_EFFORT` for sensor topics. | Set `BEST_EFFORT` for `/scan` and `/camera/*` to reduce latency |
| **Polling keyboard I/O** | `read_key()` polls stdin in a tight loop. CPU usage is higher than event-driven input. | Use `select()` with timeout (partially implemented) |

---

## Security Risks

| Risk | Description | Severity |
|------|-------------|----------|
| **No DDS security** | DDS communication is unencrypted, no authentication. Any node on the same network can publish `/cmd_vel`. | Low (simulation only) |
| **Serial port unprotected** | HMI→Arduino serial has no authentication or checksums. Malformed commands could cause unexpected behavior. | Medium (physical hardware) |
| **Hardcoded absolute paths** | Script contains absolute paths to user home directory. | Low |

---

## Technical Debt

| Item | Location | Description |
|------|----------|-------------|
| **TODO placeholders** | `package.xml` L:6-8, `setup.py` L:23-25 | Package description and license are "TODO" |
| **Unused launch file model path** | `simulation.launch.py` L:22-27 | Launch file still references stock model, not the camera model |
| **No unit tests** | `test/` directory | Only boilerplate copyright/flake8/pep257 tests, no actual unit tests |
| **Incomplete README** | `README.md` | Does not document ArrowTeleop, SensorValidator, SlamReadiness, or camera features |
| **AutonomousNavigator not in start_simulation.sh** | `start_simulation.sh` | Only teleop is started; autonomous mode has no entry in the script |
| **No RViz camera display** | `robot_view.rviz` | Camera sensor exists but not added to the RViz config by default |
| **AI detection stub** | `swacch.ino` L:196-200 | `AI:ON/OFF` command acknowledged but does nothing |

---

## Refactoring Opportunities

### High Priority

1. **Extract shared constants** — Create a `config.py` or YAML parameter file for `ROBOT_RADIUS`, `SLOW_ZONE`, speed levels, and expected sensor rates. Use ROS parameters so they can be overridden at launch time.

2. **Add cmd_vel multiplexer** — Use `twist_mux` package to prioritize teleop commands over autonomous commands, and give emergency stop highest priority.

3. **Namespace the robot** — Change all nodes and topics to use a namespace (`/swachh/`) to enable multi-robot support.

### Medium Priority

4. **Parse world file for environment_viz** — Instead of hardcoding wall positions, parse `simple_room.world` SDF at runtime to generate RViz markers automatically.

5. **Add actual unit tests** — Test collision avoidance logic, state machine transitions, and sensor validation checks in isolation.

6. **Update README** — Document all 6 nodes, camera features, speed levels, and the start_simulation.sh workflow.

### Low Priority

7. **Bridge HMI to simulation** — Create a ROS node that translates serial commands to ROS `/cmd_vel` topics, enabling the GTK HMI to control the simulated robot.

8. **Add rosbag recording** — Record `/scan`, `/cmd_vel`, `/odom`, and camera topics for offline analysis and SLAM testing.

---

## Architecture Strengths

| Strength | Description |
|----------|-------------|
| **Clean separation of concerns** | Each node has a single responsibility |
| **Graceful degradation** | `tf2_ros` import is optional (`HAS_TF2` flag) |
| **Comprehensive sensor validation** | 7-check SLAM readiness gate is thorough |
| **Safety-conscious teleop** | Proportional deceleration prevents wall collisions |
| **Dual operation modes** | Both manual (teleop) and autonomous navigation |
| **Async logging** | Non-blocking log writes don't interfere with real-time control |

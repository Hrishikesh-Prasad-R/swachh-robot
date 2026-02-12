# 06 — State Model / State Machine Diagrams

## 1. ArrowTeleop — Robot Motion States

The teleop node doesn't use explicit state constants, but has implicit states driven by `current_linear` and `current_angular` values and collision proximity.

```mermaid
stateDiagram-v2
    [*] --> Stopped: Node starts
    Stopped --> MovingForward: ↑ key + path clear
    Stopped --> MovingBackward: ↓ key
    Stopped --> TurningLeft: ← key
    Stopped --> TurningRight: → key

    MovingForward --> Decelerating: obstacle in SLOW_ZONE (0.5m)
    Decelerating --> Blocked: gap ≤ 0 (at wall)
    Decelerating --> MovingForward: obstacle moves away
    Blocked --> Stopped: automatic hard stop
    Blocked --> MovingBackward: ↓ key (reverse allowed)

    MovingForward --> Stopped: Space key / key release timeout
    MovingBackward --> Stopped: Space key / key release timeout
    TurningLeft --> Stopped: Space key / key release timeout
    TurningRight --> Stopped: Space key / key release timeout

    Stopped --> SpeedLevel1: Press '1'
    Stopped --> SpeedLevel2: Press '2'
    Stopped --> SpeedLevel3: Press '3'
```

### Collision State Parameters

| State | Condition | `linear.x` |
|-------|-----------|------------|
| **Full Speed** | `gap ≥ SLOW_ZONE (0.5m)` | `speed_levels[current]` |
| **Decelerating** | `0 < gap < SLOW_ZONE` | `base_speed × (gap / SLOW_ZONE)` |
| **Blocked** | `gap ≤ 0` | `0.0` |

Where `gap = collision_dist - ROBOT_RADIUS (0.105m)` and `SLOW_ZONE = 0.5m`.

---

## 2. AutonomousNavigator — Navigation FSM

```mermaid
stateDiagram-v2
    [*] --> FORWARD: Node starts

    FORWARD --> TURNING: min_distance < obstacle_distance
    Note right of FORWARD: cmd.linear.x = forward_speed

    TURNING --> FORWARD: turn_duration elapsed AND path clear
    TURNING --> TURNING: turn_duration elapsed BUT still blocked (30% flip direction)
    Note right of TURNING: cmd.angular.z = turn_speed × direction
```

### State Details

| State | Entry Condition | Action | Exit Condition |
|-------|----------------|--------|----------------|
| `FORWARD` | Path clear (min_dist ≥ 0.5m) | Publish `linear.x = 0.2` | Obstacle detected |
| `TURNING` | Obstacle at < 0.5m | Publish `angular.z = ±0.5` | `turn_duration` (2s) elapsed + path clear |

### Turn Direction Selection Algorithm
1. Divide scan into sectors
2. Average left-half sectors vs right-half sectors
3. Turn toward side with more open space
4. 20% random flip to prevent loops

---

## 3. Arduino System State Machine

```mermaid
stateDiagram-v2
    [*] --> READY: Power on / setup()

    READY --> OPERATING: Any command except ESTOP
    OPERATING --> OPERATING: Component ON/OFF commands
    OPERATING --> EMERGENCY_STOPPED: ESTOP command
    EMERGENCY_STOPPED --> READY: RESET command

    state OPERATING {
        [*] --> Idle
        Idle --> Moving: MOVE:FORWARD/BACKWARD/LEFT/RIGHT
        Moving --> Idle: MOVE:STOP
        Idle --> Vacuuming: VACUUM:ON
        Vacuuming --> Idle: VACUUM:OFF

        state Moving {
            [*] --> Manual
            Manual --> Autonomous: AUTO:ON
            Autonomous --> Manual: AUTO:OFF
        }
    }

    note right of EMERGENCY_STOPPED
        All motors stopped
        All subsystems OFF
        LED blinking @ 4Hz
        Only RESET accepted
    end note
```

### Arduino System State Variables

| Flag | Type | Initial | Description |
|------|------|---------|-------------|
| `vacuum_active` | bool | false | Vacuum relay state |
| `arm_active` | bool | false | Robotic arm servo state |
| `wiper_active` | bool | false | Wiper motor state |
| `uv_active` | bool | false | UV strip state |
| `autonomous_mode` | bool | false | If true, rejects manual MOVE commands |
| `emergency_stop` | bool | false | If true, rejects all commands except RESET |
| `moving` | bool | false | Motors currently active |

---

## 4. SlamReadiness — Lifecycle

```mermaid
stateDiagram-v2
    [*] --> Collecting: Node starts (t=0)

    Collecting --> TestMovement: t > 3s
    Note right of Collecting: Subscribing to /scan, /odom<br/>Accumulating data

    TestMovement --> StopTest: t > 5s
    Note right of TestMovement: Publishing cmd_vel linear.x=0.1

    StopTest --> Collecting: cmd_vel = 0.0
    Collecting --> Reporting: t ≥ 15s
    Note right of Reporting: Run 7 checks<br/>Print PASS/FAIL report

    Reporting --> [*]: SystemExit(0)
```

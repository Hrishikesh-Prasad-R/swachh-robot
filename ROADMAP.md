# Swacch Robot: Development Roadmap

**Team Size:** 4 members  
**Duration:** 12 weeks (3 months)  
**Approach:** Parallel development with weekly integration

---

## Team Roles

| Member | Role | Focus Area |
|--------|------|------------|
| **P1** | Perception Lead | YOLO, Object Detection, Camera Pipeline |
| **P2** | SLAM Lead | Visual SLAM, Mapping, Localization |
| **P3** | Navigation Lead | Path Planning, Obstacle Avoidance, Motion Control |
| **P4** | Integration Lead | ROS2, Simulation, Hardware Interface, HMI |

---

## Phase 1: Foundation (Weeks 1-3)

### Week 1: Environment Setup

| Member | Tasks | Deliverables |
|--------|-------|--------------|
| **P1** | - Install CUDA, TensorRT on dev machine<br>- Set up YOLOv8 training environment<br>- Collect/label initial dataset (50-100 images) | Working YOLO inference demo |
| **P2** | - Install ROS2 Humble<br>- Set up RTAB-Map or ORB-SLAM3<br>- Run with sample datasets (TUM, EuRoC) | SLAM running on benchmark data |
| **P3** | - Install Nav2 stack<br>- Study Nav2 architecture<br>- Run Nav2 tutorials with TurtleBot3 | Nav2 demo working in simulation |
| **P4** | - Set up Gazebo simulation<br>- Create basic robot URDF (differential drive)<br>- Set up Git repo structure | Simulated robot moving in Gazebo |

**Week 1 Integration Meeting:** Everyone demos their component working independently.

---

### Week 2: Simulation Environment

| Member | Tasks | Deliverables |
|--------|-------|--------------|
| **P1** | - Train YOLOv8-nano on vacuum-relevant classes<br>- Classes: trash, cable, shoe, chair_leg, pet<br>- Export to ONNX | Trained model (mAP > 0.6) |
| **P2** | - Configure RTAB-Map for stereo camera<br>- Test in Gazebo with simulated stereo<br>- Tune parameters for indoor environment | SLAM generating maps in simulation |
| **P3** | - Configure Nav2 costmap layers<br>- Implement basic waypoint following<br>- Test obstacle avoidance in simulation | Robot navigating around obstacles |
| **P4** | - Add stereo camera to robot URDF<br>- Create indoor environment (rooms, furniture)<br>- Add objects for YOLO testing | Complete simulation world |

**Week 2 Integration Meeting:** Test all components in same Gazebo simulation (not connected yet).

---

### Week 3: First Integration

| Member | Tasks | Deliverables |
|--------|-------|--------------|
| **P1** | - Create ROS2 node for YOLO<br>- Publish detections as ROS2 messages<br>- Subscribe to camera topic from Gazebo | YOLO node publishing detections |
| **P2** | - Connect RTAB-Map to simulated cameras<br>- Publish map and pose to ROS2 topics<br>- Test relocalization | Live mapping in simulation |
| **P3** | - Connect Nav2 to SLAM map<br>- Implement "go to pose" command<br>- Basic cleaning coverage pattern | Robot navigating using SLAM map |
| **P4** | - Write Arduino serial bridge (ROS2 ↔ Arduino)<br>- Create launch files combining all nodes<br>- Document integration steps | Full stack running together |

**Week 3 Milestone:** Robot navigates in simulation using vSLAM map. YOLO detects objects (not yet influencing navigation).

---

## Phase 2: Core Functionality (Weeks 4-6)

### Week 4: YOLO-Navigation Integration

| Member | Tasks | Deliverables |
|--------|-------|--------------|
| **P1** | - Project YOLO detections to 3D coordinates<br>- Create semantic costmap layer plugin<br>- Define cost values per object class | Detections appearing on costmap |
| **P2** | - Implement map saving/loading<br>- Add IMU fusion (if available)<br>- Improve relocalization robustness | Persistent maps, better tracking |
| **P3** | - Integrate YOLO costmap layer into Nav2<br>- Implement "avoid cable" behavior<br>- Implement "go to trash" behavior | Navigation reacting to YOLO |
| **P4** | - Add more test scenarios to simulation<br>- Create automated test scripts<br>- Performance profiling | Reproducible test suite |

---

### Week 5: Robustness & Edge Cases

| Member | Tasks | Deliverables |
|--------|-------|--------------|
| **P1** | - Add temporal filtering (require N frames)<br>- Handle false positives<br>- Optimize inference speed | Stable detections, < 50ms latency |
| **P2** | - Handle tracking loss recovery<br>- Test on challenging scenes (low texture)<br>- Add visual relocalization | SLAM recovers from failure |
| **P3** | - Implement recovery behaviors<br>- Handle "stuck" situations<br>- Add speed limits near obstacles | Robust navigation |
| **P4** | - Create HMI integration with navigation<br>- Display map and robot pose in HMI<br>- Add manual override controls | HMI shows live status |

---

### Week 6: Cleaning Behaviors

| Member | Tasks | Deliverables |
|--------|-------|--------------|
| **P1** | - Add "dirt/debris" detection class<br>- Prioritize cleaning areas with detections<br>- Log cleaning coverage | Smart dirt detection |
| **P2** | - Generate occupancy grid from SLAM<br>- Mark cleaned vs uncleaned cells<br>- Track cleaning progress | Cleaning coverage map |
| **P3** | - Implement coverage path planning<br>- Boustrophedon (lawn mower) pattern<br>- Handle room-by-room cleaning | Full coverage algorithm |
| **P4** | - Implement cleaning modes (quick, thorough)<br>- Add scheduling interface (future)<br>- System state machine | Complete mode management |

**Week 6 Milestone:** Robot performs full room cleaning in simulation with obstacle avoidance and object detection.

---

## Phase 3: Hardware Integration (Weeks 7-9)

### Week 7: Jetson Deployment

| Member | Tasks | Deliverables |
|--------|-------|--------------|
| **P1** | - Convert YOLO to TensorRT engine<br>- Benchmark on Jetson Orin Nano<br>- Optimize batch size, resolution | YOLO running at 15+ FPS |
| **P2** | - Deploy RTAB-Map on Jetson<br>- Test with real stereo camera<br>- Profile CPU/GPU usage | SLAM running on Jetson |
| **P3** | - Deploy Nav2 on Jetson<br>- Tune for Jetson compute limits<br>- Test with real camera feed | Navigation on real hardware |
| **P4** | - Set up Jetson development environment<br>- Create deployment scripts<br>- Arduino communication verified | Jetson ↔ Arduino working |

---

### Week 8: Real Robot Testing

| Member | Tasks | Deliverables |
|--------|-------|--------------|
| **P1** | - Collect real-world test images<br>- Fine-tune YOLO on real data<br>- Handle lighting variations | Model works in real world |
| **P2** | - Map real test environment<br>- Tune SLAM for real camera<br>- Test different floor types | Real environment mapped |
| **P3** | - Test navigation in real space<br>- Tune velocity limits<br>- Handle real-world obstacles | Robot navigating real space |
| **P4** | - Full system integration test<br>- Debug communication issues<br>- Create system health monitoring | Integrated robot working |

---

### Week 9: Debugging & Refinement

| Member | Tasks | Deliverables |
|--------|-------|--------------|
| **P1** | - Fix false positive issues<br>- Add confidence thresholds<br>- Handle edge cases | Reliable detection |
| **P2** | - Fix tracking loss issues<br>- Improve loop closure<br>- Handle dynamic objects | Stable localization |
| **P3** | - Fix path planning issues<br>- Improve obstacle avoidance<br>- Tune motion smoothness | Smooth navigation |
| **P4** | - Fix integration bugs<br>- Optimize resource usage<br>- Create log analysis tools | Stable system |

**Week 9 Milestone:** Robot operates autonomously in real environment for 10+ minutes without intervention.

---

## Phase 4: Advanced Features (Weeks 10-12)

### Week 10: Advanced Perception

| Member | Tasks | Deliverables |
|--------|-------|--------------|
| **P1** | - Add object tracking (SORT/DeepSORT)<br>- Track moving objects (pets, people)<br>- Predict object trajectories | Dynamic object handling |
| **P2** | - Implement multi-session mapping<br>- Map updates over time<br>- Remove moved objects from map | Adaptive mapping |
| **P3** | - Dynamic obstacle avoidance<br>- Predict and avoid moving obstacles<br>- Safety behaviors | Safe around moving objects |
| **P4** | - Battery monitoring integration<br>- Auto-return to dock behavior<br>- Power management | Smart power handling |

---

### Week 11: Advanced Navigation

| Member | Tasks | Deliverables |
|--------|-------|--------------|
| **P1** | - Room segmentation from visual data<br>- Zone classification<br>- Cleaning priority per zone | Zone-based cleaning |
| **P2** | - Multi-floor support (if applicable)<br>- Map merging<br>- Improved relocalization | Advanced mapping |
| **P3** | - Adaptive coverage based on dirt<br>- Revisit dirty areas<br>- Optimize cleaning path | Intelligent coverage |
| **P4** | - Web/mobile interface (optional)<br>- Remote monitoring<br>- Cleaning history/stats | Remote access |

---

### Week 12: Polish & Documentation

| Member | Tasks | Deliverables |
|--------|-------|--------------|
| **P1** | - Final model optimization<br>- Document detection pipeline<br>- Create training guide | Complete documentation |
| **P2** | - Document SLAM configuration<br>- Create troubleshooting guide<br>- Performance benchmarks | SLAM documentation |
| **P3** | - Document navigation setup<br>- Create parameter tuning guide<br>- Edge case documentation | Navigation docs |
| **P4** | - System architecture documentation<br>- Deployment guide<br>- Demo video/presentation | Final deliverables |

**Week 12 Milestone:** Complete system demo, documentation, and handoff.

---

## Weekly Schedule Template

```
Monday:    Individual work, standup (15 min)
Tuesday:   Individual work
Wednesday: Individual work, mid-week sync (30 min)
Thursday:  Individual work
Friday:    Integration testing, weekly review (1 hr)
Weekend:   Issue resolution (if needed)
```

---

## Git Branch Strategy

```
main
├── develop
│   ├── feature/yolo-detection (P1)
│   ├── feature/slam-mapping (P2)
│   ├── feature/navigation (P3)
│   └── feature/integration (P4)
```

**Rules:**
- Feature branches merge to `develop` on Fridays
- `develop` → `main` at end of each phase
- All merge requests require 1 review

---

## Risk Mitigation

| Risk | Mitigation | Owner |
|------|------------|-------|
| SLAM tracking loss | Add recovery behaviors, IMU fusion | P2, P3 |
| YOLO false positives | Temporal filtering, confidence thresholds | P1 |
| Compute overload | Profile early, optimize critical paths | P4 |
| Hardware delays | Develop in simulation until hardware ready | P4 |
| Integration issues | Weekly integration tests | All |

---

## Success Criteria

### Minimum Viable Product (Week 9)
- [ ] Robot maps room autonomously
- [ ] Robot navigates without collision
- [ ] YOLO detects 3+ object classes
- [ ] 10 minutes autonomous operation

### Full Product (Week 12)
- [ ] Complete room coverage cleaning
- [ ] Dynamic obstacle avoidance
- [ ] Return to dock on low battery
- [ ] Stable for 30+ minutes operation
- [ ] HMI fully functional

---

## Resources

### Documentation
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [RTAB-Map Wiki](http://wiki.ros.org/rtabmap_ros)
- [YOLOv8 Docs](https://docs.ultralytics.com/)

### Tutorials
- [Nav2 Getting Started](https://navigation.ros.org/getting_started/index.html)
- [RTAB-Map Tutorials](https://github.com/introlab/rtabmap/wiki)
- [Jetson AI Courses](https://developer.nvidia.com/embedded/learn/jetson-ai-certification-programs)

### Datasets
- [TUM RGB-D Dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset)
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

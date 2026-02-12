#!/bin/bash
# ============================================
#  🤖 Swachh Robot Simulation - SLAM-Ready Launcher
#  Run: ./start_simulation.sh
# ============================================

GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  🤖 Swachh Robot Simulation — SLAM-Ready${NC}"
echo -e "${CYAN}============================================${NC}"

# Kill old processes (ALL of them, including swachh_robot nodes)
echo -e "${YELLOW}[1/8] Cleaning up old processes...${NC}"
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
pkill -9 -f environment_viz 2>/dev/null || true
pkill -9 -f sensor_validator 2>/dev/null || true
pkill -9 -f arrow_teleop 2>/dev/null || true
pkill -9 -f slam_readiness 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
sleep 3

# Source ROS 2
echo -e "${YELLOW}[2/8] Sourcing ROS 2 Humble...${NC}"
source /opt/ros/humble/setup.bash

# Set environment
echo -e "${YELLOW}[3/8] Setting environment variables...${NC}"
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
export LIBGL_ALWAYS_SOFTWARE=1

# Create log directory
LOG_DIR="/home/bmscecse/Desktop/swachh-robot-simulation/logs"
mkdir -p "$LOG_DIR"

# Build workspace
echo -e "${YELLOW}[4/8] Building workspace...${NC}"
cd /home/bmscecse/Desktop/swachh-robot-simulation
colcon build --symlink-install --packages-select swachh_robot 2>&1 | tail -2
source install/setup.bash

# Start Gazebo
echo -e "${GREEN}[5/8] Starting Gazebo server...${NC}"
WORLD_FILE="/home/bmscecse/Desktop/swachh-robot-simulation/src/swachh_robot/worlds/simple_room.world"
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$WORLD_FILE" &
GAZEBO_PID=$!
echo "   Gazebo PID: $GAZEBO_PID"

# Wait for spawn service
echo -e "${YELLOW}   Waiting for Gazebo spawn service...${NC}"
for i in $(seq 1 60); do
    if ros2 service list 2>/dev/null | grep -q "/spawn_entity"; then
        echo -e "\n${GREEN}   ✅ Gazebo ready after ${i} seconds!${NC}"
        break
    fi
    echo -n "."
    sleep 1

    if ! kill -0 $GAZEBO_PID 2>/dev/null; then
        echo -e "\n${RED}   Gazebo crashed! Restarting...${NC}"
        gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$WORLD_FILE" &
        GAZEBO_PID=$!
    fi
done

# Final check
if ! ros2 service list 2>/dev/null | grep -q "/spawn_entity"; then
    echo -e "\n${RED}   ERROR: Gazebo spawn service not available.${NC}"
    echo -e "${RED}   Try running: pkill -9 gzserver && ./start_simulation.sh${NC}"
    kill $GAZEBO_PID 2>/dev/null
    exit 1
fi

# Spawn robot (custom model with RGB-D camera)
echo -e "${GREEN}[6/8] Spawning TurtleBot3 (with camera)...${NC}"
CUSTOM_MODEL="/home/bmscecse/Desktop/swachh-robot-simulation/src/swachh_robot/models/turtlebot3_burger_camera/model.sdf"
ros2 run gazebo_ros spawn_entity.py \
    -entity turtlebot3_burger \
    -file "$CUSTOM_MODEL" \
    -x 0.0 -y 0.0 -z 0.01

if [ $? -ne 0 ]; then
    echo -e "${RED}   Failed to spawn robot. Exiting.${NC}"
    kill $GAZEBO_PID 2>/dev/null
    exit 1
fi
sleep 2

# Start background nodes
echo -e "${GREEN}[7/8] Launching nodes...${NC}"

# Robot state publisher (publishes base_footprint → base_link → base_scan TF)
URDF_FILE="/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"
# Use xacro/urdf if available, else use the SDF description via robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(xacro /opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro 2>/dev/null || echo '')" \
    2>/dev/null &
RSP_PID=$!
echo "   Robot State Publisher: PID $RSP_PID"

ros2 run swachh_robot environment_viz 2>/dev/null &
ENV_PID=$!
echo "   Environment Visualizer: PID $ENV_PID"

ros2 run swachh_robot sensor_validator 2>/dev/null &
SENSOR_PID=$!
echo "   Sensor Validator: PID $SENSOR_PID"

# RViz — redirect ALL output to /dev/null (prevents terminal spam that breaks arrow keys)
rviz2 -d /home/bmscecse/Desktop/swachh-robot-simulation/src/swachh_robot/rviz/robot_view.rviz \
    >/dev/null 2>&1 &
RVIZ_PID=$!
echo "   RViz: PID $RVIZ_PID"

sleep 2

# SLAM Readiness Gate
echo -e "${CYAN}[8/8] Running SLAM Readiness Check...${NC}"
ros2 run swachh_robot slam_readiness
echo ""

echo ""
echo -e "${CYAN}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║          🤖 SWACHH ROBOT SIMULATION MANUAL 🤖          ║${NC}"
echo -e "${CYAN}╠══════════════════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║                                                        ║${NC}"
echo -e "${CYAN}║${NC}  ${GREEN}MOVEMENT CONTROLS${NC}                                      ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}  ─────────────────                                      ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    ↑  (UP)       Move forward                           ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    ↓  (DOWN)     Move backward                          ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    ←  (LEFT)     Turn left                              ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    →  (RIGHT)    Turn right                             ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    SPACE         Emergency stop                         ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    Q             Quit simulation                        ${CYAN}║${NC}"
echo -e "${CYAN}║                                                        ║${NC}"
echo -e "${CYAN}║${NC}  ${GREEN}SPEED LEVELS (press 1–9)${NC}                                ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}  ────────────────────────                                ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    1 = Crawl   (0.05 m/s)    6 = Fast    (0.40 m/s)     ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    2 = Slow    (0.10 m/s)    7 = Faster  (0.50 m/s)     ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    3 = Gentle  (0.15 m/s)    8 = Rush    (0.65 m/s)     ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    4 = Medium  (0.20 m/s)    9 = Max     (0.80 m/s)     ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    5 = Normal  (0.30 m/s)    ${YELLOW}[default = 5]${NC}              ${CYAN}║${NC}"
echo -e "${CYAN}║                                                        ║${NC}"
echo -e "${CYAN}║${NC}  ${GREEN}ENVIRONMENT${NC}                                              ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}  ───────────                                              ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    Room 1 (6×6m) ──corridor──▶ Room 2 (6×6m)            ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    4 obstacles (Room 1) + 3 obstacles (Room 2)          ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    Collision detection active (forward 60°)             ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    Vision cone detects obstacles with enter/exit logs   ${CYAN}║${NC}"
echo -e "${CYAN}║                                                        ║${NC}"
echo -e "${CYAN}║${NC}  ${GREEN}LOGGING (async file-only, no terminal noise)${NC}            ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}  ─────────────────────────────────────────              ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    Full log:      ${YELLOW}tail -f logs/full.log${NC}                  ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    Essential log:  ${YELLOW}tail -f logs/essential.log${NC}             ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    Filter by node: ${YELLOW}grep 'teleop' logs/full.log${NC}           ${CYAN}║${NC}"
echo -e "${CYAN}║                                                        ║${NC}"
echo -e "${CYAN}║${NC}  ${GREEN}SLAM READINESS${NC}                                           ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}  ──────────────                                           ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    Sensor validator runs in background                  ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    Checks: scan rate, TF chain, odom, data quality      ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}    Re-run gate: ${YELLOW}ros2 run swachh_robot slam_readiness${NC}     ${CYAN}║${NC}"
echo -e "${CYAN}║                                                        ║${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""

# Run teleop (foreground - blocks until Q)
ros2 run swachh_robot arrow_teleop

# Cleanup on exit
echo ""
echo -e "${YELLOW}Shutting down...${NC}"
kill $RSP_PID 2>/dev/null || true
kill $ENV_PID 2>/dev/null || true
kill $SENSOR_PID 2>/dev/null || true
kill $RVIZ_PID 2>/dev/null || true
kill $GAZEBO_PID 2>/dev/null || true
pkill -9 gzserver 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
echo -e "${GREEN}✅ Done. Goodbye!${NC}"

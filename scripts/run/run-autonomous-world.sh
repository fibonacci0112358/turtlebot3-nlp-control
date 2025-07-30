#!/bin/bash

# TurtleBot3 Autonomous Mode with Default World Runner Script
# This script launches TurtleBot3 autonomous mode using the default world

set -e

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== TurtleBot3 Autonomous Mode with Default World ===${NC}"
echo -e "${GREEN}Starting TurtleBot3 autonomous mode with default world...${NC}"

# Check if TURTLEBOT3_MODEL is set
if [ -z "$TURTLEBOT3_MODEL" ]; then
    echo -e "${YELLOW}TURTLEBOT3_MODEL not set, using default: burger${NC}"
    export TURTLEBOT3_MODEL=burger
else
    echo -e "${GREEN}Using TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL${NC}"
fi

# Check if Gazebo is already running
if pgrep -f "gzserver" > /dev/null; then
    echo -e "${YELLOW}Warning: Gazebo is already running. Consider stopping it first.${NC}"
fi

# Source ROS2 setup
echo -e "${BLUE}Sourcing ROS2 environment...${NC}"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo -e "${RED}Error: ROS2 Humble not found!${NC}"
    exit 1
fi

# Source workspace
echo -e "${BLUE}Sourcing workspace...${NC}"
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
else
    echo -e "${YELLOW}Warning: Workspace not built. Building workspace...${NC}"
    cd "$PROJECT_ROOT"
    colcon build --packages-select turtlebot3_nlp_control
    source "$PROJECT_ROOT/install/setup.bash"
fi

# Launch arguments
SLAM=${SLAM:-true}
NAVIGATION=${NAVIGATION:-true}
USE_SIM_TIME=${USE_SIM_TIME:-true}
MAP_FILE=${MAP_FILE:-""}

echo -e "${BLUE}Launch configuration:${NC}"
echo -e "  SLAM: $SLAM"
echo -e "  Navigation: $NAVIGATION"
echo -e "  Use Sim Time: $USE_SIM_TIME"
echo -e "  Map File: ${MAP_FILE:-'(none - will use SLAM)'}"

# Launch the autonomous mode
echo -e "${GREEN}Launching TurtleBot3 autonomous mode with default world...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"

ros2 launch turtlebot3_nlp_control autonomous_world_launch.py \
    use_sim_time:="$USE_SIM_TIME" \
    slam:="$SLAM" \
    navigation:="$NAVIGATION" \
    map:="$MAP_FILE"

echo -e "${GREEN}TurtleBot3 autonomous mode stopped.${NC}"
#!/bin/bash

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Default ROS distribution
ROS_DISTRO="humble"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --humble)
            ROS_DISTRO="humble"
            shift
            ;;
        --jazzy)
            ROS_DISTRO="jazzy"
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --humble    Build with ROS 2 Humble (default)"
            echo "  --jazzy     Build with ROS 2 Jazzy"
            echo "  --help, -h  Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0              # Build with ROS Humble (default)"
            echo "  $0 --jazzy      # Build with ROS Jazzy"
            echo "  $0 --humble     # Build with ROS Humble"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            echo "Run '$0 --help' for usage information"
            exit 1
            ;;
    esac
done

export ROS_DISTRO

echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}Building DimOS + ROS Autonomy Stack Docker Image${NC}"
echo -e "${GREEN}ROS Distribution: ${ROS_DISTRO}${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Clone or checkout ros-navigation-autonomy-stack with dev branch
if [ ! -d "ros-navigation-autonomy-stack" ]; then
    echo -e "${YELLOW}Cloning ros-navigation-autonomy-stack repository (dev branch)...${NC}"
    git clone -b dev git@github.com:dimensionalOS/ros-navigation-autonomy-stack.git
    echo -e "${GREEN}Repository cloned successfully!${NC}"
else
    # Directory exists, ensure we're on the dev branch
    cd ros-navigation-autonomy-stack
    CURRENT_BRANCH=$(git branch --show-current)
    if [ "$CURRENT_BRANCH" != "dev" ]; then
        echo -e "${YELLOW}Switching from ${CURRENT_BRANCH} to dev branch...${NC}"
        # Stash any local changes (e.g., auto-generated config files)
        if git stash --quiet 2>/dev/null; then
            echo -e "${YELLOW}Stashed local changes${NC}"
        fi
        git fetch origin dev
        git checkout dev
        git pull origin dev
        echo -e "${GREEN}Switched to dev branch${NC}"
    else
        echo -e "${GREEN}Already on dev branch${NC}"
    fi
    cd ..
fi

# Clone or checkout vector_perception_ros with semantic_mapping branch
if [ ! -d "vector_perception_ros" ]; then
    echo -e "${YELLOW}Cloning vector_perception_ros repository (semantic_mapping branch)...${NC}"
    git clone -b semantic_mapping https://github.com/VectorRobotics/vector_perception_ros.git
    echo -e "${GREEN}Repository cloned successfully!${NC}"
else
    # Directory exists, ensure we're on the semantic_mapping branch
    cd vector_perception_ros
    CURRENT_BRANCH=$(git branch --show-current)
    if [ "$CURRENT_BRANCH" != "semantic_mapping" ]; then
        echo -e "${YELLOW}Switching from ${CURRENT_BRANCH} to semantic_mapping branch...${NC}"
        # Stash any local changes
        if git stash --quiet 2>/dev/null; then
            echo -e "${YELLOW}Stashed local changes${NC}"
        fi
        git fetch origin semantic_mapping
        git checkout semantic_mapping
        git pull origin semantic_mapping
        echo -e "${GREEN}Switched to semantic_mapping branch${NC}"
    else
        echo -e "${GREEN}Already on semantic_mapping branch${NC}"
    fi
    cd ..
fi

if [ ! -d "unity_models" ]; then
    echo -e "${YELLOW}Using office_building_1 as the Unity environment...${NC}"
    tar -xf ../../data/.lfs/office_building_1.tar.gz
    mv office_building_1 unity_models
fi

echo ""
echo -e "${YELLOW}Building Docker image with docker compose...${NC}"
echo "This will take a while as it needs to:"
echo "  - Download base ROS ${ROS_DISTRO^} image"
echo "  - Install ROS packages and dependencies"
echo "  - Build the autonomy stack"
echo "  - Build Livox-SDK2 for Mid-360 lidar"
echo "  - Build SLAM dependencies (Sophus, Ceres, GTSAM)"
echo "  - Build vector_perception_ros for semantic mapping"
echo "  - Install Python dependencies for DimOS"
echo ""

cd ../..

docker compose -f docker/navigation/docker-compose.yml build

echo ""
echo -e "${YELLOW}Verifying build...${NC}"

# Quick smoke test - check key components
docker run --rm dimos_autonomy_stack:${ROS_DISTRO} /bin/bash -c "
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    source /opt/dimos-venv/bin/activate && \
    echo '  [✓] ROS ${ROS_DISTRO} environment' && \
    python3 -c 'import dimos; print(\"  [✓] dimos package\")' && \
    python3 -c 'import ultralytics; print(\"  [✓] ultralytics (YOLO)\")' && \
    python3 -c 'import open3d; print(\"  [✓] open3d\")' && \
    ros2 pkg list | grep -q track_anything && echo '  [✓] track_anything ROS package' && \
    ros2 pkg list | grep -q tare_planner && echo '  [✓] tare_planner ROS package' && \
    echo 'All checks passed!'
" || { echo -e "${RED}Verification failed!${NC}"; exit 1; }

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}Docker image built and verified!${NC}"
echo -e "${GREEN}Image: dimos_autonomy_stack:${ROS_DISTRO}${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "To run in SIMULATION mode:"
echo -e "${YELLOW}  ./start.sh --${ROS_DISTRO}${NC}"
echo ""
echo "To run in HARDWARE mode:"
echo "  1. Configure your hardware settings in .env file"
echo "     (copy from .env.hardware if needed)"
echo "  2. Run the hardware container:"
echo -e "${YELLOW}     ./start.sh --hardware --${ROS_DISTRO}${NC}"
echo ""
echo "The script runs in foreground. Press Ctrl+C to stop."
echo ""

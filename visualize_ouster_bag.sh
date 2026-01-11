#!/bin/bash
# Quick Start Script for Visualizing Ouster Bag Data
# Usage: ./visualize_ouster_bag.sh

echo "=========================================="
echo "Ouster Bag Visualization Quick Start"
echo "=========================================="
echo ""

# Set paths
BAG_PATH="/home/wenda/ASRL/vtr3/data/ouster_data/rosbag2_2025_04_27-18_34_00_0"
WORKSPACE="$HOME/lidar2image_ws"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}Step 1: Source the workspace${NC}"
cd $WORKSPACE
source install/setup.bash
echo -e "${GREEN}✓ Workspace sourced${NC}"
echo ""

echo -e "${BLUE}Step 2: Choose your method:${NC}"
echo ""
echo "Method 1 (Recommended): Launch everything together"
echo -e "${YELLOW}Command:${NC}"
echo "  ros2 launch lidar_to_image_ros2 play_bag_and_visualize.launch.py"
echo ""

echo "Method 2: Separate terminals for more control"
echo -e "${YELLOW}Terminal 1 - Play bag:${NC}"
echo "  ros2 bag play $BAG_PATH"
echo ""
echo -e "${YELLOW}Terminal 2 - Convert to images:${NC}"
echo "  ros2 launch lidar_to_image_ros2 cloud2image.launch.py \\"
echo "      cloud_topic:=/ouster/points \\"
echo "      sensor_model:=OS-1-64-1024 \\"
echo "      point_type:=XYZI"
echo ""

echo -e "${BLUE}Step 3: Visualize (in a new terminal):${NC}"
echo -e "${YELLOW}Option A - Image viewer:${NC}"
echo "  ros2 run rqt_image_view rqt_image_view"
echo ""
echo -e "${YELLOW}Option B - RViz2:${NC}"
echo "  ros2 run rviz2 rviz2"
echo ""

echo "=========================================="
echo "For detailed help, see:"
echo "  $WORKSPACE/src/lidar_to_image_ros2/BAG_VISUALIZATION_GUIDE.md"
echo "=========================================="

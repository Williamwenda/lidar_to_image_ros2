# Changelog for lidar_to_image_ros2

## Version 2.0.0 - ROS2 Port (January 2026)

### Major Changes
- **Complete ROS2 Migration**: Converted from ROS1 to ROS2 Jazzy
  - Changed from `ros::NodeHandle` to `rclcpp::Node`
  - Updated all publishers/subscribers to ROS2 API
  - Migrated from XML launch files to Python launch files
  - Updated parameter handling from `ros::param` to `rclcpp::Parameter`

### Dependencies Updated
- **Build System**: Catkin → ament_cmake
- **Core Libraries**: 
  - `roscpp` → `rclcpp`
  - `pcl_ros` → `pcl_conversions`
  - `cv_bridge/cv_bridge.h` → `cv_bridge/cv_bridge.hpp`
- **Message Types**: 
  - `sensor_msgs::PointCloud2::ConstPtr` → `sensor_msgs::msg::PointCloud2::SharedPtr`
  - `sensor_msgs::ImagePtr` → `sensor_msgs::msg::Image::SharedPtr`
  - `std_msgs::Header` → `std_msgs::msg::Header`

### API Changes
- Node now inherits from `rclcpp::Node`
- Changed `ROS_WARN_STREAM` → `RCLCPP_WARN`
- Changed `ROS_INFO` → `RCLCPP_INFO`
- Updated publisher API: `publisher.publish(msg)` → `publisher->publish(*msg)`
- Updated subscriber count check: `getNumSubscribers()` → `get_subscription_count()`
- Added proper shared pointer usage for messages

### Launch File Changes
- Converted from XML `.launch` to Python `.launch.py` format
- Using `launch_ros.actions.Node` instead of XML tags
- Added `DeclareLaunchArgument` for all parameters
- Using `ament_index_python` for package path resolution

### Package Structure
- Updated `package.xml` to format 3
- Removed Catkin-specific tags
- Added ROS2 build type export
- Updated all dependency declarations

### Build System
- Complete CMakeLists.txt rewrite for ament_cmake
- Changed installation directories to ROS2 standards
- Updated target dependencies to use `ament_target_dependencies`
- Added proper installation rules for launch, config, and include files

### Compatibility
- Tested on Ubuntu 24.04 with ROS2 Jazzy
- C++17 standard (upgraded from C++11)
- All original features preserved (SINGLE/GROUP/STACK output modes)
- All sensor models still supported

### Features Preserved
- Multiple LiDAR sensor support (Velodyne, Ouster)
- All point cloud types (XYZ, XYZI, XYZIR, XYZIF, XYZIFN)
- All image output modes (SINGLE, GROUP, STACK, ALL)
- Image processing options (8bpp, equalization, flip)
- PNG file saving capability
- Mossman corrections for Velodyne sensors

### Files Added
- `launch/cloud2image.launch.py` - Python launch file
- `USAGE.md` - Comprehensive usage examples
- `CHANGELOG.md` - This file

### Files Modified
- `CMakeLists.txt` - Complete rewrite for ament_cmake
- `package.xml` - Updated to format 3 for ROS2
- `src/cloud_to_image_node.cpp` - ROS2 node initialization
- `src/cloud_to_image.cpp` - ROS2 API migration
- `include/cloud_to_image.h` - Updated for rclcpp::Node inheritance
- `include/cloud_projection.h` - Updated PCL includes

### Files Unchanged (Core Logic)
- `src/cloud_projection.cpp` - Projection algorithms (with minor include fixes)
- `src/sensor_params.cpp` - Sensor parameter handling
- `src/projection_params.cpp` - Projection parameter loading
- `config/projection_params.yaml` - Sensor configurations
- All header files in `include/` (except cloud_to_image.h and cloud_projection.h)

### Known Issues
None at this time.

### Migration Notes
If you're migrating from the ROS1 version:
1. Update your launch files to Python format
2. Change topic names if needed (defaults remain the same)
3. Source the ROS2 workspace instead of ROS1
4. Use `ros2 launch` instead of `roslaunch`
5. Use `ros2 run` instead of `rosrun`

### Future Work
- Add ROS2-specific tests using ament_cmake_gtest
- Consider adding lifecycle node support
- Add component node support for composition
- Add parameter validation
- Add QoS configuration options

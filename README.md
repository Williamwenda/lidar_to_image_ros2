# lidar_to_image_ros2
Conversion from 3D LiDAR pointcloud to images - ROS2 Version

This is a ROS2 port of the original ROS1 `lidar_cloud_to_image` package.

## Overview
This package converts 3D LiDAR point clouds into various 2D image representations, making it easier to process LiDAR data using computer vision techniques.

## Features
- **Multiple LiDAR Support**: Velodyne (HDL-64-S2, HDL-64-S3, HDL-32, VLP-32, VLP-16) and Ouster (OS-1-16, OS-1-64)
- **Various Point Cloud Formats**: XYZ, XYZI, XYZIR, XYZIF, XYZIFN
- **Multiple Output Modes**: SINGLE (separate images), GROUP (stacked vertically), STACK (multi-channel), ALL (all modes)
- **Image Processing**: Histogram equalization, image flipping, 8-bit/16-bit output

## Supported Point Cloud Types
- **XYZ**: Basic 3D coordinates
- **XYZI**: 3D coordinates + Intensity
- **XYZIR**: 3D coordinates + Intensity + Ring
- **XYZIF**: 3D coordinates + Intensity + Reflectivity
- **XYZIFN**: 3D coordinates + Intensity + Reflectivity + Noise

## Installation

### Prerequisites
- Ubuntu 24.04
- ROS2 Jazzy
- Dependencies:
  - `rclcpp`
  - `sensor_msgs`
  - `pcl_ros`
  - `cv_bridge`
  - `opencv`
  - `eigen3`
  - `yaml-cpp`

### Build Instructions

1. Navigate to your ROS2 workspace:
```bash
cd ~/lidar2image_ws
```

2. Build the package:
```bash
colcon build --packages-select lidar_to_image_ros2
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Launch with Default Parameters
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py
```

### Launch with Custom Parameters
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    sensor_model:=VLP-16 \
    cloud_topic:=/velodyne_points \
    point_type:=XYZI \
    output_mode:=SINGLE
```

### Run Node Directly
```bash
ros2 run lidar_to_image_ros2 cloud2image \
    --ros-args \
    -p proj_params:=/path/to/projection_params.yaml \
    -p sensor_model:=HDL-64 \
    -p cloud_topic:=/points_raw \
    -p point_type:=XYZI
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `proj_params` | string | `config/projection_params.yaml` | Path to sensor projection parameters file |
| `cloud_topic` | string | `/points_raw` | Input point cloud topic |
| `sensor_model` | string | `HDL-64` | LiDAR sensor model |
| `point_type` | string | `XYZI` | Point cloud format |
| `depth_image_topic` | string | `/c2i_depth_image` | Output depth image topic |
| `intensity_image_topic` | string | `/c2i_intensity_image` | Output intensity image topic |
| `reflectance_image_topic` | string | `/c2i_reflectance_image` | Output reflectance image topic |
| `noise_image_topic` | string | `/c2i_noise_image` | Output noise image topic |
| `h_scale` | double | `1.0` | Horizontal scale factor |
| `v_scale` | double | `1.0` | Vertical scale factor |
| `output_mode` | string | `SINGLE` | Output mode: SINGLE, GROUP, STACK, or ALL |
| `save_images` | bool | `false` | Save images to PNG files |
| `overlapping` | int | `0` | Overlap pixels for circular wrapping |
| `8bpp` | bool | `false` | Use 8-bit output instead of 16-bit |
| `equalize` | bool | `false` | Apply histogram equalization |
| `flip` | bool | `false` | Flip images horizontally |

## Published Topics

### SINGLE Mode
- `/c2i_depth_image` (sensor_msgs/Image): Depth/range image
- `/c2i_intensity_image` (sensor_msgs/Image): Intensity image
- `/c2i_reflectance_image` (sensor_msgs/Image): Reflectance image (if available)
- `/c2i_noise_image` (sensor_msgs/Image): Noise image (if available)

### GROUP Mode
- `/c2i_group_image` (sensor_msgs/Image): All images stacked vertically

### STACK Mode
- `/c2i_stack_image` (sensor_msgs/Image): All images as separate channels

## Subscribed Topics
- `cloud_topic` (sensor_msgs/PointCloud2): Input LiDAR point cloud

## Example Visualization

View the output images using RViz2:
```bash
ros2 run rviz2 rviz2
```

Or use rqt_image_view:
```bash
ros2 run rqt_image_view rqt_image_view
```

## Configuration Files

The `config/projection_params.yaml` file contains calibration parameters for various LiDAR sensors. You can add custom sensor configurations by following the existing format.

## Credits

This work was inspired by `cloud_to_image` from I. Bogoslavskyi, C. Stachniss, University of Bonn:
https://github.com/PRBonn/cloud_to_image.git

## License

Apache License 2.0

## Maintainer

alexandrx (alexander@g.sp.m.is.nagoya-u.ac.jp)

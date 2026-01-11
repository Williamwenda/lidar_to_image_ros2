# Quick Start Guide - lidar_to_image_ros2

## Installation

```bash
cd ~/lidar2image_ws
colcon build --packages-select lidar_to_image_ros2
source install/setup.bash
```

## Verify Installation

```bash
ros2 pkg list | grep lidar_to_image
# Should output: lidar_to_image_ros2

ros2 pkg executables lidar_to_image_ros2
# Should output: lidar_to_image_ros2 cloud2image
```

## Quick Test

### 1. Launch the node with default settings
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py
```

### 2. Check published topics (in another terminal)
```bash
source ~/lidar2image_ws/install/setup.bash
ros2 topic list | grep c2i
```

You should see:
- `/c2i_depth_image`
- `/c2i_intensity_image`

### 3. Visualize with rqt_image_view
```bash
ros2 run rqt_image_view rqt_image_view
```
Select `/c2i_depth_image` or `/c2i_intensity_image` from the dropdown.

### 4. Or use RViz2
```bash
ros2 run rviz2 rviz2
```
- Click "Add" → "By topic" → Select image topic
- Or "Add" → "Image" → Set "Image Topic" manually

## Common Sensor Configurations

### Velodyne VLP-16
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    sensor_model:=VLP-16 \
    cloud_topic:=/velodyne_points
```

### Velodyne HDL-32
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    sensor_model:=HDL-32 \
    cloud_topic:=/velodyne_points
```

### Velodyne HDL-64
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    sensor_model:=HDL-64 \
    cloud_topic:=/velodyne_points
```

### Ouster OS1-64 (1024 mode)
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    sensor_model:=OS-1-64-1024 \
    cloud_topic:=/ouster/points \
    point_type:=XYZIFN
```

## Troubleshooting

### No images published?
1. Check if point cloud is being published:
   ```bash
   ros2 topic echo /points_raw --once
   ```
2. Check node status:
   ```bash
   ros2 node list
   ros2 node info /cloud2image
   ```

### Wrong sensor model?
Make sure the sensor_model matches exactly one in `config/projection_params.yaml`:
- HDL-32
- HDL-64-S2, HDL-64-S3
- VLP-16, VLP-32
- OS-1-16-0512, OS-1-16-1024, OS-1-16-2048
- OS-1-64-0512, OS-1-64-1024, OS-1-64-2048

### Images look wrong?
Try adjusting:
- `point_type`: Match your LiDAR's point format (XYZ, XYZI, XYZIR, XYZIF, XYZIFN)
- `8bpp:=true`: Use 8-bit images instead of 16-bit
- `equalize:=true`: Apply histogram equalization for better visualization

## Next Steps

- Read [README.md](README.md) for detailed documentation
- Check [USAGE.md](USAGE.md) for more examples
- Review [CHANGELOG.md](CHANGELOG.md) for migration notes from ROS1

## Getting Help

If you encounter issues:
1. Check your ROS2 installation: `ros2 doctor`
2. Verify workspace setup: `echo $ROS_DISTRO` (should show "jazzy")
3. Check the sensor configuration in `config/projection_params.yaml`
4. Enable debug output: `ros2 launch lidar_to_image_ros2 cloud2image.launch.py --log-level debug`

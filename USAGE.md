# Usage Examples for lidar_to_image_ros2

## Basic Usage

### 1. Source the workspace
```bash
source ~/lidar2image_ws/install/setup.bash
```

### 2. Run with default parameters (VLP-16)
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py sensor_model:=VLP-16
```

### 3. Run with HDL-64 sensor
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    sensor_model:=HDL-64 \
    cloud_topic:=/velodyne_points \
    point_type:=XYZI
```

### 4. Run with Ouster sensor
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    sensor_model:=OS-1-64-1024 \
    cloud_topic:=/ouster/points \
    point_type:=XYZIFN \
    output_mode:=ALL
```

## Advanced Examples

### Save images to disk
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    sensor_model:=HDL-32 \
    save_images:=true \
    8bpp:=true \
    equalize:=true
```

### Use GROUP output mode for visualization
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    sensor_model:=VLP-16 \
    output_mode:=GROUP
```

### Use STACK output mode (multi-channel)
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    sensor_model:=HDL-64 \
    output_mode:=STACK \
    point_type:=XYZIF
```

### With image scaling and flipping
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    sensor_model:=HDL-64 \
    h_scale:=2.0 \
    v_scale:=2.0 \
    flip:=true
```

### Custom projection parameters file
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    proj_params:=/path/to/custom/projection_params.yaml \
    sensor_model:=CustomSensor
```

## Running the node directly (without launch file)

```bash
ros2 run lidar_to_image_ros2 cloud2image \
    --ros-args \
    -p proj_params:=$(ros2 pkg prefix lidar_to_image_ros2)/share/lidar_to_image_ros2/config/projection_params.yaml \
    -p sensor_model:=VLP-16 \
    -p cloud_topic:=/velodyne_points \
    -p point_type:=XYZI \
    -p output_mode:=SINGLE
```

## Visualize the output

### Using RViz2
```bash
# In terminal 1 - run the node
ros2 launch lidar_to_image_ros2 cloud2image.launch.py

# In terminal 2 - run RViz2
ros2 run rviz2 rviz2
# Then add Image display and select the topic:
# /c2i_depth_image or /c2i_intensity_image
```

### Using rqt_image_view
```bash
# In terminal 1 - run the node
ros2 launch lidar_to_image_ros2 cloud2image.launch.py

# In terminal 2 - view images
ros2 run rqt_image_view rqt_image_view
```

## Check available topics

```bash
ros2 topic list | grep c2i
```

Expected output (with SINGLE mode):
```
/c2i_depth_image
/c2i_intensity_image
```

## Echo topic info

```bash
ros2 topic info /c2i_depth_image
ros2 topic echo /c2i_depth_image --once
```

## Supported Sensor Models

The following sensor models are pre-configured in `config/projection_params.yaml`:
- HDL-32
- HDL-64-S2
- HDL-64-S3
- VLP-16
- VLP-32
- OS-1-16-0512
- OS-1-16-1024
- OS-1-16-2048
- OS-1-64-0512
- OS-1-64-1024
- OS-1-64-2048

## Tips

1. **Performance**: Use 8bpp mode for faster processing if you don't need 16-bit precision
2. **Visualization**: Use GROUP mode to see all image types stacked vertically
3. **Computer Vision**: Use STACK mode to get multi-channel images for ML applications
4. **Storage**: Enable save_images to debug or create datasets

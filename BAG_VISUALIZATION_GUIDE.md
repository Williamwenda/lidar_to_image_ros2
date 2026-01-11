# Visualizing ROS2 Bag with Ouster Data

## Your Bag Information
- **Location**: `/home/wenda/ASRL/vtr3/data/ouster_data/rosbag2_2025_04_27-18_34_00_0/`
- **Topic**: `/ouster/points`
- **Type**: `sensor_msgs/msg/PointCloud2`
- **Messages**: 4223 frames

## Quick Start - Three Methods

### Method 1: Using the Combined Launch File (Recommended)

This automatically plays the bag and converts to images:

```bash
# Source your workspace
cd ~/lidar2image_ws
source install/setup.bash

# Launch everything at once
ros2 launch lidar_to_image_ros2 play_bag_and_visualize.launch.py

# With custom parameters
ros2 launch lidar_to_image_ros2 play_bag_and_visualize.launch.py \
    sensor_model:=OS-1-64-1024 \
    point_type:=XYZI \
    rate:=1.0 \
    output_mode:=SINGLE
```

Then in another terminal:
```bash
# Visualize with rqt_image_view
ros2 run rqt_image_view rqt_image_view
# Select /c2i_depth_image or /c2i_intensity_image
```

---

### Method 2: Separate Terminals (More Control)

**Terminal 1 - Play the bag:**
```bash
cd ~/lidar2image_ws
source install/setup.bash

ros2 bag play /home/wenda/ASRL/vtr3/data/ouster_data/rosbag2_2025_04_27-18_34_00_0
```

**Terminal 2 - Launch cloud2image converter:**
```bash
cd ~/lidar2image_ws
source install/setup.bash

ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    cloud_topic:=/ouster/points \
    sensor_model:=OS-1-64-1024 \
    point_type:=XYZI \
    8bpp:=true
```

**Terminal 3 - Visualize images:**
```bash
source ~/lidar2image_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

---

### Method 3: Using RViz2 for 3D + Image Visualization

**Terminal 1 - Play bag and convert:**
```bash
cd ~/lidar2image_ws
source install/setup.bash
ros2 launch lidar_to_image_ros2 play_bag_and_visualize.launch.py
```

**Terminal 2 - Launch RViz2:**
```bash
source ~/lidar2image_ws/install/setup.bash
ros2 run rviz2 rviz2
```

In RViz2:
1. Click **"Add"** → **"By topic"**
2. Add **PointCloud2** display for `/ouster/points` (3D view)
3. Add **Image** display for `/c2i_depth_image` (range image)
4. Add **Image** display for `/c2i_intensity_image` (intensity image)
5. Set Fixed Frame to the frame in your point cloud (usually "os_sensor" or "lidar")

---

## Bag Playback Options

### Play at different speeds:
```bash
# Play at half speed
ros2 bag play <bag_path> --rate 0.5

# Play at 2x speed
ros2 bag play <bag_path> --rate 2.0

# Loop continuously
ros2 bag play <bag_path> --loop

# Start paused (press space to play)
ros2 bag play <bag_path> --start-paused
```

### Play specific duration:
```bash
# Play only first 10 seconds
ros2 bag play <bag_path> --duration 10

# Skip first 5 seconds
ros2 bag play <bag_path> --start-offset 5
```

---

## Determining Your Ouster Sensor Model

First, inspect your bag to determine the exact sensor configuration:

```bash
# Get bag info
ros2 bag info /home/wenda/ASRL/vtr3/data/ouster_data/rosbag2_2025_04_27-18_34_00_0

# Check point cloud format
ros2 bag play /home/wenda/ASRL/vtr3/data/ouster_data/rosbag2_2025_04_27-18_34_00_0 &
ros2 topic echo /ouster/points --once | head -50
```

### Ouster Sensor Models Available:
- **OS-1-16-0512** (16 channels, 512 columns)
- **OS-1-16-1024** (16 channels, 1024 columns)
- **OS-1-16-2048** (16 channels, 2048 columns)
- **OS-1-64-0512** (64 channels, 512 columns)
- **OS-1-64-1024** (64 channels, 1024 columns) ← Most common
- **OS-1-64-2048** (64 channels, 2048 columns)

### Point Cloud Types for Ouster:
- **XYZI** - Basic: X, Y, Z, Intensity
- **XYZIR** - With ring: X, Y, Z, Intensity, Ring
- **XYZIF** - With reflectivity: X, Y, Z, Intensity, Reflectivity
- **XYZIFN** - Full: X, Y, Z, Intensity, Reflectivity, Noise

If unsure, start with `OS-1-64-1024` and `XYZI`.

---

## Troubleshooting

### Issue: "No images appearing"

**Check if topics are publishing:**
```bash
ros2 topic list | grep c2i
ros2 topic hz /c2i_depth_image
ros2 topic echo /c2i_depth_image --once
```

**Check if bag is playing:**
```bash
ros2 topic list | grep ouster
ros2 topic hz /ouster/points
```

### Issue: "Wrong sensor model error"

Edit the config file to see available models:
```bash
cat ~/lidar2image_ws/src/lidar_to_image_ros2/config/projection_params.yaml | grep "name:"
```

Try different models:
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    cloud_topic:=/ouster/points \
    sensor_model:=OS-1-64-2048
```

### Issue: "Images look distorted"

Try different point types:
```bash
# Try XYZIR if XYZI doesn't work
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    cloud_topic:=/ouster/points \
    sensor_model:=OS-1-64-1024 \
    point_type:=XYZIR
```

### Issue: "Bag playback too fast/slow"

Adjust the rate:
```bash
ros2 launch lidar_to_image_ros2 play_bag_and_visualize.launch.py rate:=0.5
```

---

## Advanced: Save Images to Files

Enable image saving:
```bash
ros2 launch lidar_to_image_ros2 cloud2image.launch.py \
    cloud_topic:=/ouster/points \
    sensor_model:=OS-1-64-1024 \
    save_images:=true \
    8bpp:=true
```

Images will be saved as PNG files with timestamps in the current directory.

---

## Complete Example Command

Here's a complete command that should work for most Ouster OS-1-64 sensors:

```bash
# Terminal 1 - All in one
cd ~/lidar2image_ws
source install/setup.bash
ros2 launch lidar_to_image_ros2 play_bag_and_visualize.launch.py \
    bag_path:=/home/wenda/ASRL/vtr3/data/ouster_data/rosbag2_2025_04_27-18_34_00_0 \
    sensor_model:=OS-1-64-1024 \
    point_type:=XYZI \
    output_mode:=ALL \
    rate:=1.0

# Terminal 2 - Visualize
source ~/lidar2image_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

---

## Quick Reference Commands

```bash
# 1. Source workspace
cd ~/lidar2image_ws && source install/setup.bash

# 2. Launch (choose one):
# Option A: Combined launch
ros2 launch lidar_to_image_ros2 play_bag_and_visualize.launch.py

# Option B: Separate bag play
ros2 bag play /home/wenda/ASRL/vtr3/data/ouster_data/rosbag2_2025_04_27-18_34_00_0

# 3. Visualize (in another terminal after sourcing):
ros2 run rqt_image_view rqt_image_view
# or
ros2 run rviz2 rviz2
```

That's it! You should now see your Ouster point cloud data converted to range and intensity images.

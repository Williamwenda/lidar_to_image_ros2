# Image Size Configuration for Ouster LiDAR

## Understanding Image Dimensions

The converted LiDAR image dimensions are determined by:
- **Height (rows)**: Vertical angular resolution = `vertical_span / vertical_step`
- **Width (cols)**: Horizontal angular resolution = `horizontal_span / horizontal_step`

## Why Different Heights?

### Original Configuration (64 rows)
```yaml
OS-1-64-1024:
  vertical_span: 16.6° to -16.6° (33.2° total)
  vertical_step: 0.51875°
  rows: 33.2 / 0.51875 ≈ 64 rows
```

This matches the **physical laser beams** (64 channels on OS-1-64), but Ouster's default visualization uses **interpolation** to create a taller, smoother image.

### HIGH Configuration (128 rows) - NEW!
```yaml
OS-1-64-1024-HIGH:
  vertical_span: 16.6° to -16.6° (33.2° total)
  vertical_step: 0.259375°  # Half the step = double the rows
  rows: 33.2 / 0.259375 ≈ 128 rows
```

This creates a **taller image** that matches Ouster's default visualization by interpolating between laser beams.

## Available Sensor Models

### Standard Resolution (matches physical laser count)
- `OS-1-16-0512` - 16 rows × 512 cols
- `OS-1-16-1024` - 16 rows × 1024 cols
- `OS-1-16-2048` - 16 rows × 2048 cols
- `OS-1-64-0512` - 64 rows × 512 cols
- `OS-1-64-1024` - 64 rows × 1024 cols
- `OS-1-64-2048` - 64 rows × 2048 cols

### High Resolution (doubled vertical, matches Ouster default viz)
- `OS-1-64-1024-HIGH` - **128 rows** × 1024 cols ⭐ (Recommended for matching Ouster default)
- `OS-1-64-2048-HIGH` - **128 rows** × 2048 cols

## How to Use

### Method 1: Launch with HIGH resolution (Default Now)
```bash
ros2 launch lidar_to_image_ros2 play_bag_and_visualize.launch.py
# Uses OS-1-64-1024-HIGH by default
```

### Method 2: Explicitly Specify Model
```bash
ros2 launch lidar_to_image_ros2 play_bag_and_visualize.launch.py \
    sensor_model:=OS-1-64-1024-HIGH
```

### Method 3: Use Standard Resolution
```bash
ros2 launch lidar_to_image_ros2 play_bag_and_visualize.launch.py \
    sensor_model:=OS-1-64-1024
```

## Comparison

| Configuration | Rows | Visual Result |
|---------------|------|---------------|
| `OS-1-64-1024` | 64 | Shorter, matches physical beams |
| `OS-1-64-1024-HIGH` | 128 | **Taller, matches Ouster default** ⭐ |

## Image Size Examples

### OS-1-64-1024 (Standard)
- **Size**: 64 rows × 1024 cols
- **Aspect Ratio**: ~16:1
- **Use Case**: Compact representation, faster processing

### OS-1-64-1024-HIGH (High Resolution)
- **Size**: 128 rows × 1024 cols
- **Aspect Ratio**: ~8:1
- **Use Case**: Better visualization, matches Ouster SDK default
- **Trade-off**: Slightly more computation

### OS-1-64-2048-HIGH (Highest Resolution)
- **Size**: 128 rows × 2048 cols
- **Aspect Ratio**: ~16:1 (but with 2x horizontal detail)
- **Use Case**: Maximum detail, best for analysis
- **Trade-off**: Most computation

## How Interpolation Works

The HIGH variants use **angular interpolation**:
1. Point clouds are projected to the finer angular grid
2. Each pixel represents a smaller angular slice
3. Multiple points may fall into same pixel (uses closest point)
4. Results in smoother, taller images
5. No artificial data is created - just finer binning

## Recommendation

**For visual comparison with Ouster's default:** Use `OS-1-64-1024-HIGH`

This will give you images that match the height of Ouster Studio's default visualization while maintaining accurate geometric projection.

## Performance Impact

The HIGH variants have minimal performance impact:
- ✅ Same point cloud processing
- ✅ Slightly larger image buffers (128 vs 64 rows)
- ✅ Still runs at full LiDAR frame rate (10-20 Hz)
- ⚠️ 2x more pixels to normalize/display (128 vs 64 rows)

On modern hardware, this difference is negligible.

## Creating Custom Resolutions

To create your own resolution, edit `config/projection_params.yaml`:

```yaml
- name: YOUR-CUSTOM-NAME
  params:
    vertical_span:
      start_angle: 16.6
      end_angle: -16.6
      step: YOUR_STEP_SIZE  # Smaller = more rows
    horizontal_span:
      start_angle: -180
      end_angle: 180
      step: YOUR_HORIZ_STEP  # Smaller = more columns
    scan_direction: CCW
```

**Calculate rows:** `(start_angle - end_angle) / step`
- For 64 rows: `33.2 / 0.51875 = 64`
- For 128 rows: `33.2 / 0.259375 = 128`
- For 256 rows: `33.2 / 0.1296875 = 256`

Then rebuild:
```bash
colcon build --packages-select lidar_to_image_ros2
```

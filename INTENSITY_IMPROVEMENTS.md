# Intensity Image Quality Improvements

## Problems Identified

### Issue 1: Incorrect Intensity Scaling for Velodyne/Generic Sensors
**Original Code:**
```cpp
uint16_t intensity = (uint16_t)((point.intensity/255.0)*65535.0);
```

**Problem:** This assumed intensity values were in the range 0-255 (like Velodyne), but **Ouster LiDAR** intensity values are already in a much larger range (typically 0-10000 or higher).

**Solution:** Direct clamping without rescaling:
```cpp
uint16_t intensity = (uint16_t)std::min(std::max(point.intensity, 0.0f), 65535.0f);
```

### Issue 2: Poor Normalization Strategy
**Original Code:**
```cpp
cv::normalize(_intensity_image, mono_img, min_range, max_range, cv::NORM_MINMAX, mode);
```

**Problem:** `cv::NORM_MINMAX` uses the absolute min/max values, which means:
- A few bright outliers make everything else dark
- A few dark outliers compress the useful range
- Poor contrast in the resulting image

**Solution:** Percentile-based normalization (2nd to 98th percentile):
```cpp
// Calculate 2nd and 98th percentile
std::sort(intensity_values.begin(), intensity_values.end());
double min_val = intensity_values[size * 0.02];  // 2nd percentile
double max_val = intensity_values[size * 0.98];  // 98th percentile

// Normalize using this robust range
_intensity_image.convertTo(mono_img, mode, 
    max_range / (max_val - min_val), 
    -min_val * max_range / (max_val - min_val));
```

## Benefits of the New Approach

### 1. Preserves Original Ouster Intensity Values
- No artificial scaling that loses information
- Maintains the sensor's native dynamic range
- Works correctly for Ouster's higher bit-depth intensity

### 2. Robust to Outliers
- Ignores the brightest 2% and darkest 2% of pixels
- Focuses on the main distribution of intensity values
- Prevents a few outliers from ruining the entire image contrast

### 3. Better Contrast
- Utilizes the full 8-bit or 16-bit output range effectively
- Similar to Ouster's default visualization
- More detail visible in both bright and dark regions

### 4. Handles Zero Values Properly
- Excludes zero (no-return) pixels from normalization
- Keeps background as true black
- Only normalizes valid returns

## Additional Parameter Options

You can further improve the images by enabling histogram equalization:

```bash
ros2 launch lidar_to_image_ros2 play_bag_and_visualize.launch.py \
    equalize:=true \
    8bpp:=true
```

**Histogram Equalization Benefits:**
- Further enhances contrast
- Spreads out frequently occurring intensity values
- Can help in low-contrast scenes

**Note:** Equalization works best with 8bpp mode enabled.

## Comparison

| Aspect | Old Method | New Method |
|--------|------------|------------|
| Intensity Source | Scaled from 255 | Direct from sensor |
| Normalization | Min-Max (outlier sensitive) | Percentile-based (robust) |
| Contrast | Poor (compressed by outliers) | Good (focuses on main data) |
| Ouster Compatibility | Poor | Excellent |
| Velodyne Compatibility | Good | Good |

## Testing Your Changes

1. **Restart your launch:**
```bash
# Stop the current process (Ctrl+C)
cd ~/lidar2image_ws
source install/setup.bash
ros2 launch lidar_to_image_ros2 play_bag_and_visualize.launch.py
```

2. **View the improved intensity image:**
```bash
ros2 run rqt_image_view rqt_image_view
# Select /c2i_intensity_image
```

You should now see:
- ✅ Much better contrast
- ✅ More details visible
- ✅ Similar quality to Ouster's default image
- ✅ Less noise/graininess
- ✅ Better dynamic range utilization

## Advanced: Fine-Tuning Percentiles

If you want to adjust the percentiles for different scenarios, edit `src/cloud_to_image.cpp` lines 280-281:

```cpp
size_t idx_low = intensity_values.size() * 0.02;   // Change 0.02 for different low percentile
size_t idx_high = intensity_values.size() * 0.98;  // Change 0.98 for different high percentile
```

**Suggestions:**
- **More aggressive outlier removal:** Use 0.05 and 0.95 (5th-95th percentile)
- **Less aggressive:** Use 0.01 and 0.99 (1st-99th percentile)
- **Keep all data:** Use 0.0 and 1.0 (min-max, like original)

## Performance Note

The percentile-based normalization is slightly more computationally expensive than simple min-max, but:
- Still runs in real-time for typical LiDAR rates (10-20 Hz)
- The sorting is only done on non-zero pixels
- The visual quality improvement is worth the small overhead

## For Other Sensor Types

This improvement works well for:
- ✅ **Ouster OS-1/OS-2** (primary target)
- ✅ **Velodyne HDL/VLP series** (still works fine)
- ✅ **Any LiDAR with high-dynamic-range intensity**

The code automatically adapts to the intensity range of your sensor.

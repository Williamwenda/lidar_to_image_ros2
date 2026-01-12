# Gap Filling Improvements for Aeva Lidar Intensity Images

## Why Do Black Bars (Gaps) Exist?

### Root Causes:

1. **Unorganized Point Clouds**
   - Aeva FMCW lidar produces **unorganized point clouds**, unlike mechanical spinning lidars (Ouster)
   - Not every pixel in the image grid (64×601) receives a point
   - Only ~40-45% of pixels have actual lidar returns

2. **Angular Quantization**
   - 3D points are projected to 2D grid by binning angles
   - Continuous angles → discrete pixel positions
   - Some angle bins receive no points = black pixels

3. **Sparse Scanning Pattern**
   - FMCW scanning doesn't guarantee uniform spatial coverage
   - Natural gaps between laser beams
   - Occlusions and low reflectivity surfaces

4. **Motion During Scan**
   - Even with motion compensation, discretization creates gaps
   - Vehicle motion can stretch or compress point distribution

### Why Original Gap Filling Blurred Details:

The previous **5-pass aggressive method** caused over-smoothing:
- Pass 1-2: Filled 1-8 pixel horizontal gaps
- Pass 3: Vertical interpolation
- Pass 4-5: **8-neighbor averaging** (2 passes)
  - Averaged up to 8 surrounding pixels
  - Multiple passes spread values far from originals
  - **Lost sharp intensity transitions** (edges, details)
  - Made images look "blurred" or "diffused"

## Improved Gap Filling Strategy

### Three Methods Available:

#### **Method 0: Conservative (Default - Recommended for Aeva)**
- **Goal**: Preserve maximum detail, fill only obvious gaps
- Fills 1-2 pixel gaps with strict similarity (15% threshold)
- Uses **depth guidance** - avoids mixing different surfaces
- **Pros**: Preserves sharp details, no blurring
- **Cons**: Some black bars remain (acceptable trade-off)

#### **Method 1: Moderate**
- **Goal**: Balance between detail and coverage
- Fills up to 3-5 pixel gaps with relaxed similarity (25-40% threshold)
- Still uses depth guidance
- **Pros**: Reduces more gaps while keeping most details
- **Cons**: Slight smoothing on edges

#### **Method 2: Aggressive (Original)**
- **Goal**: Maximum gap coverage
- 5-pass method with 8-neighbor averaging
- **Pros**: Fills almost all gaps
- **Cons**: Blurs details significantly

### Key Improvements:

1. **Depth-Guided Filling** (Methods 0-1 only)
   ```cpp
   // Check if neighboring pixels are on the same surface
   if (|depth_end - depth_start| / max_depth < 10%)  // Strict!
       interpolate_intensity();
   ```
   - Prevents mixing intensity from different objects
   - Preserves edges at depth discontinuities

2. **Stricter Similarity Thresholds**
   - Conservative: 15% intensity difference maximum
   - Moderate: 25-40% intensity difference
   - **Only interpolates between similar values**

3. **Horizontal Priority**
   - Lidar scans are more coherent horizontally
   - Vertical gaps more likely to be real occlusions
   - Reduces false interpolation

4. **Fewer Passes**
   - Methods 0-1 use only 1-2 passes (not 5!)
   - Prevents accumulation of smoothing artifacts

## Usage

### Launch with Conservative Filling (Default):
```bash
ros2 launch lidar_to_image_ros2 aeva_bag_visualize.launch.py
# Equivalent to: fill_gaps_method:=0
```

### Try Moderate Filling:
```bash
ros2 launch lidar_to_image_ros2 aeva_bag_visualize.launch.py fill_gaps_method:=1
```

### Use Aggressive Filling (Original):
```bash
ros2 launch lidar_to_image_ros2 aeva_bag_visualize.launch.py fill_gaps_method:=2
```

### Disable Gap Filling:
```bash
ros2 launch lidar_to_image_ros2 aeva_bag_visualize.launch.py fill_gaps:=false
```

## Expected Results

### With `fill_gaps_method:=0` (Conservative):
- ✅ **Sharp intensity details preserved**
- ✅ Edges and boundaries remain clear
- ✅ No blurring or over-smoothing
- ⚠️ Some small black bars remain (1-3 pixels)
- **Best for analysis requiring high detail**

### With `fill_gaps_method:=1` (Moderate):
- ✅ Good detail preservation
- ✅ Fewer black bars than conservative
- ⚠️ Slight smoothing at some edges
- **Best for visualization and general use**

### With `fill_gaps_method:=2` (Aggressive):
- ✅ Almost no black bars
- ❌ Blurred details (same as before)
- ❌ Over-smoothed edges
- **Use only if complete coverage is critical**

## Technical Details

### Conservative Method Algorithm:
```
For each row:
    For each gap (start_pixel → end_pixel):
        gap_size = end_pixel - start_pixel - 1
        
        if gap_size <= 2:  // Only tiny gaps
            if |intensity_end - intensity_start| / max < 15%:  // Similar intensity
                if depth_guidance_available:
                    if |depth_end - depth_start| / max < 10%:  // Same surface
                        linear_interpolate()
                else:
                    linear_interpolate()
```

### Why This Works Better:
1. **Preserves real gaps** - Large gaps likely indicate real occlusions or surface boundaries
2. **Depth-aware** - Doesn't blend different objects together
3. **Minimal smoothing** - Only 1-2 passes, no neighbor averaging
4. **Data-driven** - Only interpolates when confident values are similar

## Recommendations

For **Aeva lidar intensity analysis**:
- Start with **Method 0 (conservative)** - Default
- If black bars are too distracting, try **Method 1 (moderate)**
- Avoid Method 2 unless you need video/presentation quality images where some blur is acceptable

The small remaining black bars with Method 0 are **actual data gaps**, not artifacts. They represent:
- Areas with no lidar returns (occlusions, low reflectivity)
- Boundaries between different surfaces
- Natural sparsity of FMCW scanning

**Preserving these gaps is better than blurring real intensity variations!**

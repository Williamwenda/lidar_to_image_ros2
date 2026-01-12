
#include "cloud_projection.h"
#include "pcl_point_types.h"
#include <string>
#include <vector>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <pcl/io/pcd_io.h>

// This work was inspired on cloud_projection from I. Bogoslavskyi, C. Stachniss, University of Bonn 
// https://github.com/PRBonn/cloud_to_image.git
// The original license copyright is as follows:
//------------------------------------
// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.
//------------------------------------

namespace cloud_to_image {

const std::vector<float> MOOSMAN_CORRECTIONS{
    {0.02587499999999987,   -0.0061250000000001581, 0.031874999999999876,
     0.001874999999999849,  0.029874999999999874,   -0.1961250000000001,
     0.049874999999999892,  -0.034125000000000183,  0.0038749999999998508,
     0.0058749999999998526, 0.035874999999999879,   -0.064124999999999988,
     0.035874999999999879,  0.001874999999999849,   -0.024125000000000174,
     -0.062124999999999986, 0.039874999999999883,   -0.020125000000000171,
     0.075874999999999915,  -0.024125000000000174,  -0.0041250000000001563,
     -0.058124999999999982, -0.032125000000000181,  -0.058124999999999982,
     0.021874999999999867,  -0.032125000000000181,  0.059874999999999901,
     -0.04412499999999997,  0.075874999999999915,   -0.0041250000000001563,
     0.021874999999999867,  0.0058749999999998526,  -0.036125000000000185,
     -0.022125000000000172, -0.0041250000000001563, -0.058124999999999982,
     -0.026125000000000176, -0.030125000000000179,  0.045874999999999888,
     0.035874999999999879,  -0.026125000000000176,  0.041874999999999885,
     -0.086125000000000007, -0.060124999999999984,  0.031874999999999876,
     -0.010125000000000162, -0.024125000000000174,  -0.048124999999999973,
     -0.038125000000000187, 0.039874999999999883,   -0.026125000000000176,
     0.037874999999999881,  -0.020125000000000171,  0.051874999999999893,
     -0.014125000000000165, 0.019874999999999865,   -0.0021250000000001545,
     0.027874999999999872,  0.0058749999999998526,  0.021874999999999867,
     0.023874999999999869,  0.085874999999999702,   0.085874999999999702,
     0.11587499999999995}};

CloudProjection::CloudProjection(const SensorParams& params)
    : _params(params)
{
  if (!_params.valid()) {
    throw std::runtime_error("sensor parameters not valid for projection.");
  }
  clearData();
  clearCorrections();
}

void CloudProjection::setMotionCompensation(bool enable,
                                            const Eigen::Vector3f& linear_velocity,
                                            const Eigen::Vector3f& angular_velocity)
{
  _use_motion_compensation = enable;
  _linear_velocity = linear_velocity;
  _angular_velocity = angular_velocity;
  
  if (enable) {
    std::cout << "Motion compensation enabled:" << std::endl;
    std::cout << "  Linear velocity: [" << linear_velocity.x() << ", " 
              << linear_velocity.y() << ", " << linear_velocity.z() << "] m/s" << std::endl;
    std::cout << "  Angular velocity: [" << angular_velocity.x() << ", " 
              << angular_velocity.y() << ", " << angular_velocity.z() << "] rad/s" << std::endl;
  }
}

void CloudProjection::clearData() 
{
  _data = PointMatrix(_params.cols(), PointColumn(_params.rows()));
  _depth_image = cv::Mat::zeros(_params.rows(), _params.cols(), CV_32FC1);
  _intensity_image = cv::Mat::zeros(_params.rows(), _params.cols(), CV_16UC1);
  _reflectance_image = cv::Mat::zeros(_params.rows(), _params.cols(), CV_16UC1);
  _noise_image = cv::Mat::zeros(_params.rows(), _params.cols(), CV_16UC1);
}

void CloudProjection::loadMossmanCorrections()
{
  _corrections = MOOSMAN_CORRECTIONS;
}

void CloudProjection::clearCorrections()
{
  _corrections.clear();
}

void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>(cloud);
  for (size_t index = 0; index < cloud->points.size(); ++index) {
    const auto& point = cloud->points[index];
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);    
    if (dist_to_sensor < 0.01f) {
      continue;
    }
    auto angle_rows = Angle::fromRadians(asin(point.z / dist_to_sensor));
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);
    size_t bin_cols = this->_params.colFromAngle(angle_cols);
    // adding point pointer
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);
    if (current_written_depth <= 0.0f || dist_to_sensor < current_written_depth) {
      // keep the closest point per pixel to reduce occlusion artifacts
      current_written_depth = dist_to_sensor;
    }
  }
  fixDepthSystematicErrorIfNeeded();
}

void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZI>::ConstPtr>(cloud);
  
  // Track intensity range and projection success for debugging
  float min_intensity = std::numeric_limits<float>::max();
  float max_intensity = std::numeric_limits<float>::lowest();
  int non_zero_intensity_points = 0;
  int projected_points = 0;
  int skipped_too_close = 0;
  
  for (size_t index = 0; index < cloud->points.size(); ++index) {
    const auto& point = cloud->points[index];
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    
    if (dist_to_sensor < 0.01f) {
      skipped_too_close++;
      continue;
    }
    
    // Track intensity range (including negative values)
    if (point.intensity != 0.0f) {
      min_intensity = std::min(min_intensity, point.intensity);
      max_intensity = std::max(max_intensity, point.intensity);
      non_zero_intensity_points++;
    }
    
    // Scale intensity for visualization
    // Aeva FMCW lidar uses negative dBm values (typically -20 to -120 dBm)
    uint16_t intensity;
    
    if (point.intensity < 0.0f) {
      // Negative dBm values: map to 0-65535
      // Typical range: -20 (strong) to -120 (weak)
      // Invert so stronger returns = brighter pixels
      float dbm_val = -point.intensity;  // Make positive
      // Clamp to reasonable range and normalize
      dbm_val = std::max(20.0f, std::min(dbm_val, 120.0f));
      // Map [20, 120] to [65535, 0] (inverted - stronger signal = brighter)
      intensity = (uint16_t)(((120.0f - dbm_val) / 100.0f) * 65535.0f);
    } else if (point.intensity > 0.0f && point.intensity < 100.0f) {
      // Small positive values - scale up
      intensity = (uint16_t)std::min(point.intensity * 655.0f, 65535.0f);
    } else if (point.intensity >= 100.0f) {
      // Already in reasonable range
      intensity = (uint16_t)std::min(point.intensity, 65535.0f);
    } else {
      // Zero intensity - keep as zero
      intensity = 0;
    }
    
    auto angle_rows = Angle::fromRadians(asin(point.z / dist_to_sensor));
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);
    size_t bin_cols = this->_params.colFromAngle(angle_cols);
    
    // adding point pointer
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);
    auto& current_written_intensity = this->_intensity_image.template at<uint16_t>(bin_rows, bin_cols);
    if (current_written_depth <= 0.0f || dist_to_sensor < current_written_depth) {
      // keep the closest point per pixel to reduce occlusion artifacts
      current_written_depth = dist_to_sensor;
      current_written_intensity = intensity;
      projected_points++;
    }
  }
  
  // Log statistics for debugging
  static int frame_count = 0;
  std::cout << "Frame " << frame_count++ << " - Total points: " << cloud->points.size() 
            << ", Projected: " << projected_points 
            << ", Skipped (too close): " << skipped_too_close << std::endl;
  if (non_zero_intensity_points > 0) {
    std::cout << "  Intensity range: [" << min_intensity << ", " << max_intensity 
              << "] from " << non_zero_intensity_points << " non-zero points" << std::endl;
  } else {
    std::cout << "  WARNING: ALL intensity values are ZERO!" << std::endl;
  }
  
  fixDepthSystematicErrorIfNeeded();
}

void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZIT>::ConstPtr& cloud) 
{
  // XYZIT points with time offset for motion compensation support (Aeva lidar)
  
  size_t projected_points = 0;
  size_t skipped_too_close = 0;
  
  // Track actual angle ranges to verify FOV
  float min_vertical_angle_deg = 90.0f;
  float max_vertical_angle_deg = -90.0f;
  float min_horizontal_angle_deg = 180.0f;
  float max_horizontal_angle_deg = -180.0f;
  
  for (size_t index = 0; index < cloud->points.size(); ++index) {
    const auto& point = cloud->points[index];
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
      continue;
    }
    
    Eigen::Vector3f point_vec(point.x, point.y, point.z);
    
    // Motion compensation: Transform point backwards in time to scan start
    // This undoes the vehicle motion that occurred during the scan
    if (_use_motion_compensation) {
      float t = static_cast<float>(point.time_offset_ns) * 1e-9f;
      if (std::abs(t) > 1e-6f) {
        // The sensor moved during the scan. We need to undo this motion.
        // Transform: P_compensated = R^(-1) * (P_measured - v*t)
        // where R is the rotation that occurred during time t
        
        // Step 1: Remove translation that occurred during time t
        Eigen::Vector3f point_translated = point_vec - _linear_velocity * t;
        
        // Step 2: Apply inverse rotation using Rodrigues' formula
        Eigen::Vector3f w = _angular_velocity;
        float w_norm = w.norm();
        
        if (w_norm > 1e-6f) {
          // Rotation angle (negative for inverse rotation)
          float angle = -w_norm * t;
          Eigen::Vector3f axis = w / w_norm;
          
          // Rodrigues' rotation formula
          Eigen::Matrix3f K;
          K << 0.0f, -axis.z(), axis.y(),
               axis.z(), 0.0f, -axis.x(),
               -axis.y(), axis.x(), 0.0f;
          
          Eigen::Matrix3f rotation_inv = Eigen::Matrix3f::Identity()
                                        + std::sin(angle) * K
                                        + (1.0f - std::cos(angle)) * (K * K);
          
          point_vec = rotation_inv * point_translated;
        } else {
          // No rotation, just translation compensation
          point_vec = point_translated;
        }
      }
    }

    float dist_to_sensor = point_vec.norm();
    if (dist_to_sensor < 0.01f) {
      skipped_too_close++;
      continue;
    }
    
    // Handle negative dBm intensity values (Aeva lidar)
    uint16_t intensity = 0;
    if (point.intensity != 0.0f) {
      if (point.intensity < 0.0f) {
        // Aeva dBm: typically -20 to -120 dBm
        float dbm_val = -point.intensity;  // Make positive
        dbm_val = std::max(20.0f, std::min(dbm_val, 120.0f));  // Clamp to [20, 120]
        // Map [20, 120] dBm to [65535, 0] - stronger signal (lower dBm magnitude) = brighter
        intensity = (uint16_t)(((120.0f - dbm_val) / 100.0f) * 65535.0f);
      } else {
        // Positive intensity (non-Aeva lidars)
        float intensity_val = std::max(0.0f, std::min(point.intensity, 255.0f));
        intensity = (uint16_t)(intensity_val * 256.0f);
      }
    }
    
    Angle angle_rows = Angle::fromRadians(std::atan2(point_vec.z(), std::sqrt(point_vec.x() * point_vec.x() + point_vec.y() * point_vec.y())));
    Angle angle_cols = Angle::fromRadians(std::atan2(point_vec.y(), point_vec.x()));
    
    // Track actual angle ranges
    float vert_angle_deg = angle_rows.toDegrees();
    float horiz_angle_deg = angle_cols.toDegrees();
    min_vertical_angle_deg = std::min(min_vertical_angle_deg, vert_angle_deg);
    max_vertical_angle_deg = std::max(max_vertical_angle_deg, vert_angle_deg);
    min_horizontal_angle_deg = std::min(min_horizontal_angle_deg, horiz_angle_deg);
    max_horizontal_angle_deg = std::max(max_horizontal_angle_deg, horiz_angle_deg);
    
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);
    size_t bin_cols = this->_params.colFromAngle(angle_cols);
    
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);
    auto& current_written_intensity = this->_intensity_image.template at<uint16_t>(bin_rows, bin_cols);
    if (current_written_depth <= 0.0f || dist_to_sensor < current_written_depth) {
      // keep the closest point per pixel to reduce occlusion artifacts
      current_written_depth = dist_to_sensor;
      current_written_intensity = intensity;
      projected_points++;
    }
  }
  
  // Log statistics including actual FOV observed
  static int frame_count = 0;
  std::cout << "Frame " << frame_count++ << " (XYZIT with motion compensation)" << std::endl;
  std::cout << "  Total points: " << cloud->points.size() 
            << ", Projected: " << projected_points 
            << ", Skipped (too close): " << skipped_too_close << std::endl;
  std::cout << "  Actual vertical FOV: [" << min_vertical_angle_deg << "°, " 
            << max_vertical_angle_deg << "°] (range: " 
            << (max_vertical_angle_deg - min_vertical_angle_deg) << "°)" << std::endl;
  std::cout << "  Actual horizontal FOV: [" << min_horizontal_angle_deg << "°, " 
            << max_horizontal_angle_deg << "°] (range: " 
            << (max_horizontal_angle_deg - min_horizontal_angle_deg) << "°)" << std::endl;
  std::cout << "  Configured vertical FOV: [" << this->_params.v_start_angle().toDegrees() 
            << "°, " << this->_params.v_end_angle().toDegrees() << "°]" << std::endl;
  std::cout << "  Configured horizontal FOV: [" << this->_params.h_start_angle().toDegrees() 
            << "°, " << this->_params.h_end_angle().toDegrees() << "°]" << std::endl;
  
  fixDepthSystematicErrorIfNeeded();
}

void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZIR>::ConstPtr>(cloud);
  for (size_t index = 0; index < cloud->points.size(); ++index) {
    const auto& point = cloud->points[index];
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    // For Ouster, intensity is typically already in a reasonable range
    uint16_t intensity = (uint16_t)std::min(std::max(point.intensity, 0.0f), 65535.0f);
    uint16_t ring = point.ring;
    
    if (dist_to_sensor < 0.01f) {
      continue;
    }
    
    // Use ring field directly for row (eliminates gaps from angle quantization)
    // For Ouster, ring field directly corresponds to the beam/row number
    size_t bin_rows = ring;
    
    // Still use angle for column to handle wrap-around correctly
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));
    size_t bin_cols = this->_params.colFromAngle(angle_cols);
    
    // Bounds check for ring field
    if (bin_rows >= this->_params.rows()) {
      continue;  // Skip invalid ring values
    }
    
    // adding point pointer
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);
    auto& current_written_intensity = this->_intensity_image.template at<uint16_t>(bin_rows, bin_cols);
    if (current_written_depth <= 0.0f || dist_to_sensor < current_written_depth) {
      // keep the closest point per pixel to reduce occlusion artifacts
      current_written_depth = dist_to_sensor;
      current_written_intensity = intensity;
    }
  }
  fixDepthSystematicErrorIfNeeded();
}

void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZIF>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZIF>::ConstPtr>(cloud);
  for (size_t index = 0; index < cloud->points.size(); ++index) {
    const auto& point = cloud->points[index];
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    uint16_t intensity = point.intensity;
    uint16_t reflectivity = point.reflectivity;
    if (dist_to_sensor < 0.01f) {
      continue;
    }
    auto angle_rows = Angle::fromRadians(asin(point.z / dist_to_sensor));
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);
    size_t bin_cols = this->_params.colFromAngle(angle_cols);
    // adding point pointer
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);
    auto& current_written_intensity = this->_intensity_image.template at<uint16_t>(bin_rows, bin_cols);
    auto& current_written_reflectivity = this->_reflectance_image.template at<uint16_t>(bin_rows, bin_cols);
    if (current_written_depth <= 0.0f || dist_to_sensor < current_written_depth) {
      // keep the closest point per pixel to reduce occlusion artifacts
      current_written_depth = dist_to_sensor;
      current_written_intensity = intensity;
      current_written_reflectivity = reflectivity;
    }
  }
  fixDepthSystematicErrorIfNeeded();
}

void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZIFN>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZIFN>::ConstPtr>(cloud);
  for (size_t index = 0; index < cloud->points.size(); ++index) {
    const auto& point = cloud->points[index];
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    uint16_t intensity = point.intensity;
    uint16_t reflectivity = point.reflectivity;
    uint16_t noise = point.noise;
    if (dist_to_sensor < 0.01f) {
      continue;
    }
    auto angle_rows = Angle::fromRadians(asin(point.z / dist_to_sensor));
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);
    size_t bin_cols = this->_params.colFromAngle(angle_cols);
    // adding point pointer
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);
    auto& current_written_intensity = this->_intensity_image.template at<uint16_t>(bin_rows, bin_cols);
    auto& current_written_reflectivity = this->_reflectance_image.template at<uint16_t>(bin_rows, bin_cols);
    auto& current_written_noise = this->_noise_image.template at<uint16_t>(bin_rows, bin_cols);
    if (current_written_depth <= 0.0f || dist_to_sensor < current_written_depth) {
      // keep the closest point per pixel to reduce occlusion artifacts
      current_written_depth = dist_to_sensor;
      current_written_intensity = intensity;
      current_written_reflectivity = reflectivity;
      current_written_noise = noise;
    }
  }
  fixDepthSystematicErrorIfNeeded();
}


pcl::PointCloud<pcl::PointXYZI>::Ptr CloudProjection::fromImage(const cv::Mat& depth_image) {
  checkImageAndStorage(depth_image);
  cloneDepthImage(depth_image);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  for (int r = 0; r < depth_image.rows; ++r) {
    for (int c = 0; c < depth_image.cols; ++c) {
      if (depth_image.at<float>(r, c) < 0.0001f) {
        continue;
      }
      pcl::PointXYZ point;
      unprojectPoint(depth_image, r, c, point);
      pcl::PointXYZI point2;
      point2.x = point.x;
      point2.y = point.y;
      point2.z = point.z;
      point2.intensity = 0;
      cloud->points.push_back(point2);
      this->at(r, c).points().push_back(cloud->points.size() - 1);
    }
  }
  return cloud;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr CloudProjection::fromImage(const cv::Mat& depth_image, const cv::Mat& intensity_image) 
{
  checkImageAndStorage(depth_image);
  cloneDepthImage(depth_image);
  checkImageAndStorage(intensity_image);
  cloneDepthImage(intensity_image);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  for (int r = 0; r < depth_image.rows; ++r) {
    for (int c = 0; c < depth_image.cols; ++c) {
      if (depth_image.at<float>(r, c) < 0.0001f) {
        continue;
      }
      pcl::PointXYZI point;
      unprojectPoint(depth_image, intensity_image, r, c, point);
      cloud->points.push_back(point);
      this->at(r, c).points().push_back(cloud->points.size() - 1);
    }
  }
  return cloud;
}

void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const int row, const int col, pcl::PointXYZ& point) const 
{
  float depth = depth_image.at<float>(row, col);
  Angle angle_z = this->_params.angleFromRow(row);
  Angle angle_xy = this->_params.angleFromCol(col);
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());
  point.z = depth * sinf(angle_z.val());
}

void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const int row, const int col, pcl::PointXYZI& point) const 
{
  float depth = depth_image.at<float>(row, col);
  Angle angle_z = this->_params.angleFromRow(row);
  Angle angle_xy = this->_params.angleFromCol(col);
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());
  point.z = depth * sinf(angle_z.val());
  point.intensity =  intensity_image.at<uint16_t>(row, col);
}

void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const int row, const int col, pcl::PointXYZIR& point) const 
{
  float depth = depth_image.at<float>(row, col);
  Angle angle_z = this->_params.angleFromRow(row);
  Angle angle_xy = this->_params.angleFromCol(col);
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());
  point.z = depth * sinf(angle_z.val());
  point.intensity =  intensity_image.at<uint16_t>(row, col);
  point.ring = row; //ring is handled same as row number
}

void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const cv::Mat& reflectance_image, const int row, const int col, pcl::PointXYZIF& point) const
{
  float depth = depth_image.at<float>(row, col);
  Angle angle_z = this->_params.angleFromRow(row);
  Angle angle_xy = this->_params.angleFromCol(col);
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());
  point.z = depth * sinf(angle_z.val());
  point.intensity =  intensity_image.at<uint16_t>(row, col);
  point.reflectivity =  reflectance_image.at<uint16_t>(row, col);
}

void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const cv::Mat& reflectance_image, const cv::Mat& noise_image, const int row, const int col, pcl::PointXYZIFN& point) const
{
  float depth = depth_image.at<float>(row, col);
  Angle angle_z = this->_params.angleFromRow(row);
  Angle angle_xy = this->_params.angleFromCol(col);
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());
  point.z = depth * sinf(angle_z.val());
  point.intensity =  intensity_image.at<uint16_t>(row, col);
  point.reflectivity =  reflectance_image.at<uint16_t>(row, col);
  point.noise =  noise_image.at<uint16_t>(row, col);
}


template <typename T>
void CloudProjection::checkCloudAndStorage(const T& cloud) {
  if (this->_data.size() < 1) {
    throw std::length_error("_data size is < 1");
  }
  if (cloud->points.size()==0) {
    throw std::runtime_error("cannot fill from cloud: no points");
  }
}

void CloudProjection::checkImageAndStorage(const cv::Mat& image) {
  if (image.type() != CV_32F && image.type() != CV_16U) {
    throw std::runtime_error("wrong image format");
  }
  if (this->_data.size() < 1) {
    throw std::length_error("_data size is < 1");
  }
  if (this->rows() != static_cast<size_t>(image.rows) ||
      this->cols() != static_cast<size_t>(image.cols)) {
    throw std::length_error("_data dimensions do not correspond to image ones");
  }
}

void CloudProjection::fixDepthSystematicErrorIfNeeded() {
  if (_depth_image.rows < 1) {
    //fprintf(stderr, "[INFO]: depth image of wrong size, not correcting depth\n");
    return;
  }
  if (_intensity_image.rows < 1) {
    //fprintf(stderr, "[INFO]: intensity image of wrong size, not correcting depth\n");
    return;
  }
  if (_reflectance_image.rows < 1) {
    //fprintf(stderr, "[INFO]: reflectance image of wrong size, not correcting depth\n");
    return;
  }
  if (_noise_image.rows < 1) {
    //fprintf(stderr, "[INFO]: noise image of wrong size, not correcting depth\n");
    return;
  }  
  if (_corrections.size() != static_cast<size_t>(_depth_image.rows)) {
    //fprintf(stderr, "[INFO]: Not correcting depth data.\n");
    return;
  }
  for (int r = 0; r < _depth_image.rows; ++r) {
    auto correction = _corrections[r];
    for (int c = 0; c < _depth_image.cols; ++c) {
      if (_depth_image.at<float>(r, c) < 0.001f) {
        continue;
      }
      _depth_image.at<float>(r, c) -= correction;
    }
  }
}


pcl::PointCloud<pcl::PointXYZI>::Ptr CloudProjection::readKittiCloud(const std::string& filename) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  std::fstream file(filename.c_str(), std::ios::in | std::ios::binary);
  if (file.good()) {
    file.seekg(0, std::ios::beg);
    for (int i = 0; file.good() && !file.eof(); ++i) {
      pcl::PointXYZI point;
      file.read(reinterpret_cast<char*>(&point.x), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.y), sizeof(float));
      file.read(reinterpret_cast<char*>(&point.z), sizeof(float));
      // ignore intensity
      file.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));
      cloud->push_back(point);
    }
    file.close();
  }
  if (!cloud->points.size()) {
    throw std::runtime_error("point cloud is empty, cannot load");
  } 
  return cloud;
}

cv::Mat CloudProjection::fixKITTIDepth(const cv::Mat& original)  
{
  cv::Mat fixed = original;
  for (int r = 0; r < fixed.rows; ++r) {
    auto correction = MOOSMAN_CORRECTIONS[r];
    for (int c = 0; c < fixed.cols; ++c) {
      if (fixed.at<float>(r, c) < 0.001f) {
        continue;
      }
      fixed.at<float>(r, c) -= correction;
    }
  }
  return fixed;
}

cv::Mat CloudProjection::cvMatFromDepthPNG(const std::string& filename) 
{
  cv::Mat depth_image = cv::imread(filename, cv::IMREAD_ANYDEPTH /*CV_LOAD_IMAGE_ANYDEPTH*/);
  depth_image.convertTo(depth_image, CV_32F);
  if (depth_image.type() != CV_32F && depth_image.type() != CV_16U) {
    throw std::runtime_error("wrong image format, cannot load");
  }
  if (depth_image.rows < 1 || depth_image.cols < 1) {
    throw std::runtime_error("wrong image format, cannot load");
  }

  depth_image /= 500.;
  return fixKITTIDepth(depth_image);
}

void CloudProjection::cloudToPCDFile(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, const std::string& filename)
{
  if (!cloud->points.size()) {
    throw std::runtime_error("point cloud is empty, cannot save");
  }  
  pcl::io::savePCDFileBinary(filename, *cloud);
}

void CloudProjection::cvMatToDepthPNG(const cv::Mat& image, const std::string& filename)
{
  if (image.type() != CV_32F && image.type() != CV_16U) {
    throw std::runtime_error("wrong image format for depth information, cannot save when saving image for file \"" + filename + "\"");
  }
  cvMatToColorPNG(image, filename);
}

void CloudProjection::cvMatToColorPNG(const cv::Mat& image, const std::string& filename)
{
  if (image.rows < 1 || image.cols < 1) {
    throw std::runtime_error("wrong image format, cannot save when saving image for file \"" + filename + "\"");
  }  
  try {
      cv::imwrite(filename, image);
  }
  catch (std::runtime_error& ex) {
      fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
  }
}


}  // namespace cloud_to_image

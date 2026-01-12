#include "cloud_to_image.h"
#include "projection_params.h"
#include "cloud_projection.h"
#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <Eigen/Core>

#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace cloud_to_image
{

CloudToImage::CloudToImage():
	Node("cloud2image"),
	_has_depth_image(false),
	_has_intensity_image(false),
	_has_reflectance_image(false),
	_has_noise_image(false),
	_save_images(false),
	_8bpp(false),
	_equalize(false),
	_flip(false),
	_fill_gaps(true),  // Enable gap filling by default
	_fill_gaps_method(0),  // Conservative gap filling by default
	_overlapping(0),
	_output_mode(ImageOutputMode::SINGLE),
	_motion_compensation(false),
	_has_odom_data(false),
	_record_video(false),
	_video_fps(10),
	_intensity_gamma(0.6),
	_intensity_percentile_low(0.005),
	_intensity_percentile_high(0.998)
{
	_depth_image = cv::Mat::zeros(1, 1, CV_32FC1);
	_intensity_image = cv::Mat::zeros(1, 1, CV_16UC1);
	_reflectance_image = cv::Mat::zeros(1, 1, CV_16UC1);
	_noise_image = cv::Mat::zeros(1, 1, CV_16UC1);	
	_group_image = cv::Mat::zeros(1, 1, CV_16UC1);
	_stack_image = cv::Mat::zeros(1, 1, CV_16UC3);
	
	_latest_linear_velocity = Eigen::Vector3f::Zero();
	_latest_angular_velocity = Eigen::Vector3f::Zero();
}

CloudToImage::~CloudToImage() 
{
	// Close video writers if recording
	if (_video_writer_depth.isOpened()) {
		_video_writer_depth.release();
		RCLCPP_INFO(this->get_logger(), "Closed depth video writer");
	}
	if (_video_writer_intensity.isOpened()) {
		_video_writer_intensity.release();
		RCLCPP_INFO(this->get_logger(), "Closed intensity video writer");
	}
	
	if (_cloud_proj) {
		delete _cloud_proj;
	}
}

template <typename T>
T CloudToImage::getParam(const std::string& param_name, T default_value)
{
	this->declare_parameter(param_name, default_value);
	T value;
	if (!this->get_parameter(param_name, value)) {
		RCLCPP_WARN(this->get_logger(), "Parameter '%s' not found, defaults to '%s'", 
			param_name.c_str(), std::to_string(default_value).c_str());
		value = default_value;
	}
	return value;
}

template <>
std::string CloudToImage::getParam(const std::string& param_name, std::string default_value)
{
	this->declare_parameter(param_name, default_value);
	std::string value;
	if (!this->get_parameter(param_name, value)) {
		RCLCPP_WARN(this->get_logger(), "Parameter '%s' not found, defaults to '%s'", 
			param_name.c_str(), default_value.c_str());
		value = default_value;
	}
	return value;
}

void CloudToImage::init()
{
	std::string config_filename;

	//read all parameters
	config_filename = getParam("proj_params", std::string(""));

	_cloud_topic = getParam("cloud_topic", std::string("/points_raw"));
	_sensor_model = getParam("sensor_model", std::string("VLP-16"));
	_point_type = getParam("point_type", std::string("XYZI"));

	_depth_image_topic = getParam("depth_image_topic", std::string("/c2i_depth_image"));
	_intensity_image_topic = getParam("intensity_image_topic", std::string("/c2i_intensity_image"));
	_reflectance_image_topic = getParam("reflectance_image_topic", std::string("/c2i_reflectance_image"));
	_noise_image_topic = getParam("noise_image_topic", std::string("/c2i_noise_image"));

	_horizontal_scale = getParam("h_scale", 1.0);
	_vertical_scale = getParam("v_scale", 1.0);
	std::string output_mode = getParam("output_mode", std::string("SINGLE"));

	_save_images = getParam("save_images", false);
	_overlapping = getParam("overlapping", 0);

	_8bpp = getParam("8bpp", false);	
	_equalize = getParam("equalize", false);
	_flip = getParam("flip", false);
	_fill_gaps = getParam("fill_gaps", true);  // Enable gap filling by default
	_fill_gaps_method = getParam("fill_gaps_method", 0);  // 0=conservative, 1=moderate, 2=aggressive
	
	// Video recording parameters
	_record_video = getParam("record_video", false);
	_video_output_path = getParam("video_output_path", std::string("./lidar_video"));
	_video_fps = getParam("video_fps", 10);
	
	// Intensity visualization parameters (adjustable)
	_intensity_gamma = getParam("intensity_gamma", 0.6);
	_intensity_percentile_low = getParam("intensity_percentile_low", 0.005);
	_intensity_percentile_high = getParam("intensity_percentile_high", 0.998);
	
	_equalize = _equalize && _8bpp; //no equalization for non 8bpp images

	if (boost::iequals(output_mode, "SINGLE")) {
		_output_mode = ImageOutputMode::SINGLE;
	} else if (boost::iequals(output_mode, "GROUP")) {
		_output_mode = ImageOutputMode::GROUP;
	} else if (boost::iequals(output_mode, "STACK")) {
		_output_mode = ImageOutputMode::STACK;
	} else if (boost::iequals(output_mode, "ALL")) {
		_output_mode = ImageOutputMode::ALL;
	} else {
		throw std::runtime_error("image output mode \"" + output_mode + "\" not supported");
	}

	if (boost::iequals(_point_type, "XYZ")) {  //no intensity (ex. Velodyne, Ouster)
		_has_depth_image = true;
		_output_mode = ImageOutputMode::SINGLE; //only depth images, therefore only SINGLE mode
	} else if (boost::iequals(_point_type, "XYZI")) { //with intensity (ex. Velodyne, Ouster)
		_has_depth_image = true;
		_has_intensity_image = true;
	} else if (boost::iequals(_point_type, "XYZIR")) { //with intensity (ex. Velodyne, Ouster)
		_has_depth_image = true;
		_has_intensity_image = true;
	} else if (boost::iequals(_point_type, "XYZIF")) { //with intensity and reflectance (ex. Ouster)
		_has_depth_image = true;
		_has_intensity_image = true;
		_has_reflectance_image = true;
	} else if (boost::iequals(_point_type, "XYZIFN")) { //with intensity and reflectance and noise (ex. Ouster)
		_has_depth_image = true;
		_has_intensity_image = true;
		_has_reflectance_image = true;
		_has_noise_image = true;
	} else if (boost::iequals(_point_type, "XYZIT")) { //with intensity and time offset (ex. Aeva)
		_has_depth_image = true;
		_has_intensity_image = true;
	} else {
		throw std::runtime_error("point type \"" + _point_type + "\" not supported");
	}

	//verifies the configuration file
	std::ifstream config(config_filename.c_str());
	if (!config.good()) {
		throw std::runtime_error("Projection parameters file \"" + config_filename + "\" does not exist or invalid path");	
	}

	//loads projection parameters from config file
	ProjectionParams proj_params;
	proj_params.loadFromFile(config_filename);
	//verifies the sensor model
	if (!proj_params.sensorExists(_sensor_model)) {
		throw std::runtime_error("Sensor model \"" + _sensor_model + "\" does not exist in configuration file");	
	}
	//creates the cloud projection for the sensor model
	_cloud_proj = new CloudProjection(*proj_params[_sensor_model]);

	// Motion compensation setup - read from odometry topic instead of static parameters
	_motion_compensation = getParam("motion_compensation", false);
	_odom_topic = getParam("odom_topic", std::string("/vtr/odometry"));
	
	// Initialize motion compensation with zero velocity (will be updated from odometry)
	_cloud_proj->setMotionCompensation(_motion_compensation, 
		Eigen::Vector3f::Zero(), 
		Eigen::Vector3f::Zero());
		
	if (_motion_compensation) {
		if (!boost::iequals(_point_type, "XYZIT")) {
			RCLCPP_WARN(this->get_logger(), 
				"motion_compensation enabled but point_type is not XYZIT; time offsets will be ignored");
		}
		RCLCPP_INFO(this->get_logger(), 
			"Motion compensation enabled, subscribing to odometry topic: %s", _odom_topic.c_str());
	}

	//Mossman corrections seem to help on Velodyne
	if (boost::iequals(_sensor_model, "HDL-64") || boost::iequals(_sensor_model, "HDL-32") || boost::iequals(_sensor_model, "VLP-16")) {
		_cloud_proj->loadMossmanCorrections();
	}

	//create the subscribers and publishers
	_sub_PointCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		_cloud_topic, 10, 
		std::bind(&CloudToImage::pointCloudCallback, this, std::placeholders::_1));
	
	// Subscribe to odometry if motion compensation is enabled
	if (_motion_compensation) {
		_sub_Odometry = this->create_subscription<nav_msgs::msg::Odometry>(
			_odom_topic, 10,
			std::bind(&CloudToImage::odometryCallback, this, std::placeholders::_1));
	}
	
	//publishers
	if (_output_mode == ImageOutputMode::GROUP || _output_mode == ImageOutputMode::ALL) {
		_pub_GroupImage = this->create_publisher<sensor_msgs::msg::Image>(
			std::string("/c2i_group_image"), 10);
	} 
	if (_output_mode == ImageOutputMode::STACK || _output_mode == ImageOutputMode::ALL) {
		_pub_StackImage = this->create_publisher<sensor_msgs::msg::Image>(
			std::string("/c2i_stack_image"), 10);
	} 
	if (_output_mode == ImageOutputMode::SINGLE || _output_mode == ImageOutputMode::ALL) {
		_pub_DepthImage = this->create_publisher<sensor_msgs::msg::Image>(
			_depth_image_topic, 10);
		_pub_IntensityImage = this->create_publisher<sensor_msgs::msg::Image>(
			_intensity_image_topic, 10);
		_pub_ReflectanceImage = this->create_publisher<sensor_msgs::msg::Image>(
			_reflectance_image_topic, 10);
		_pub_NoiseImage = this->create_publisher<sensor_msgs::msg::Image>(
			_noise_image_topic, 10);
	} 
	
	RCLCPP_INFO(this->get_logger(), "Cloud2Image node initialized successfully");
}

void CloudToImage::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	// Extract linear and angular velocity from odometry message
	_latest_linear_velocity = Eigen::Vector3f(
		msg->twist.twist.linear.x,
		msg->twist.twist.linear.y,
		msg->twist.twist.linear.z
	);
	
	_latest_angular_velocity = Eigen::Vector3f(
		msg->twist.twist.angular.x,
		msg->twist.twist.angular.y,
		msg->twist.twist.angular.z
	);
	
	_has_odom_data = true;
	
	// Update motion compensation with latest velocity
	if (_motion_compensation) {
		_cloud_proj->setMotionCompensation(true, _latest_linear_velocity, _latest_angular_velocity);
	}
}

void CloudToImage::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
	// Warn if motion compensation is enabled but no odometry data received yet
	if (_motion_compensation && !_has_odom_data) {
		static bool warned = false;
		if (!warned) {
			RCLCPP_WARN(this->get_logger(), 
				"Motion compensation enabled but no odometry data received yet on topic: %s", 
				_odom_topic.c_str());
			warned = true;
		}
	}
	
	//clear any previous projection data
	_cloud_proj->clearData();

	if (boost::iequals(_point_type, "XYZ")) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*input, *cloud_ptr);
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZ> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	} else if (boost::iequals(_point_type, "XYZIR")) { 
		pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZIR>);	
		pcl::fromROSMsg(*input, *cloud_ptr);
		const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZIR> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	} else if (boost::iequals(_point_type, "XYZI")) { 
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);	
		pcl::fromROSMsg(*input, *cloud_ptr);
		const pcl::PointCloud<pcl::PointXYZI>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZI> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	} else if (boost::iequals(_point_type, "XYZIF")) { 
		pcl::PointCloud<pcl::PointXYZIF>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZIF>);	
		pcl::fromROSMsg(*input, *cloud_ptr);
		const pcl::PointCloud<pcl::PointXYZIF>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZIF> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	} else if (boost::iequals(_point_type, "XYZIFN")) { 
		pcl::PointCloud<pcl::PointXYZIFN>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZIFN>);	
		pcl::fromROSMsg(*input, *cloud_ptr);
		const pcl::PointCloud<pcl::PointXYZIFN>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZIFN> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	} else if (boost::iequals(_point_type, "XYZIT")) { 
		pcl::PointCloud<pcl::PointXYZIT>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZIT>);	
		pcl::fromROSMsg(*input, *cloud_ptr);
		const pcl::PointCloud<pcl::PointXYZIT>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZIT> >);
		_cloud_proj->initFromPoints(c_cloud_ptr);
	}

	//get a local copy of each image
	_depth_image = _cloud_proj->depth_image();
	_intensity_image = _cloud_proj->intensity_image();
	_reflectance_image = _cloud_proj->reflectance_image();
	_noise_image = _cloud_proj->noise_image();
	
	// Fill gaps in images to remove black bars/artifacts
	if (_fill_gaps) {
		if (_has_depth_image) {
			fillGaps(_depth_image, _fill_gaps_method);
		}
		if (_has_intensity_image) {
			// Use depth-guided filling for intensity to preserve edges
			fillGaps(_intensity_image, _fill_gaps_method, _has_depth_image ? &_depth_image : nullptr);
		}
		if (_has_reflectance_image) {
			fillGaps(_reflectance_image, _fill_gaps_method, _has_depth_image ? &_depth_image : nullptr);
		}
		if (_has_noise_image) {
			fillGaps(_noise_image, _fill_gaps_method);
		}
	}
	
	if (_overlapping) {
		cv::Mat left_roi = _depth_image.colRange(cv::Range(0,_overlapping));
		cv::hconcat(_depth_image, left_roi, _depth_image);
		left_roi = _intensity_image.colRange(cv::Range(0,_overlapping));
		cv::hconcat(_intensity_image, left_roi, _intensity_image);
		left_roi = _reflectance_image.colRange(cv::Range(0,_overlapping));
		cv::hconcat(_reflectance_image, left_roi, _reflectance_image);
		left_roi = _noise_image.colRange(cv::Range(0,_overlapping));
		cv::hconcat(_noise_image, left_roi, _noise_image);
	}

	publishImages(input->header);
}

void CloudToImage::publishImages(const std_msgs::msg::Header& header)
{
	auto mode = CV_16UC1;
	auto min_range = 0.0;
	auto max_range = 65535.0;
	auto encoding = sensor_msgs::image_encodings::MONO16;
	if (_8bpp) {
		mode = CV_8UC1;
		max_range = 255;
		encoding = sensor_msgs::image_encodings::MONO8;
	}
	
	if (_output_mode == ImageOutputMode::SINGLE || _output_mode == ImageOutputMode::ALL) {

		if (_has_depth_image && _pub_DepthImage->get_subscription_count() > 0) {
			//depth image is float, normalize it for visualization
			cv::Mat mono_img = cv::Mat(_depth_image.size(), mode);
			cv::normalize(_depth_image, mono_img, min_range, max_range, cv::NORM_MINMAX, mode);
			cv::resize(mono_img, mono_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, cv::INTER_LINEAR);
			if (_equalize) cv::equalizeHist( mono_img, mono_img );
			if (_flip) cv::flip( mono_img, mono_img, 1);
			sensor_msgs::msg::Image::SharedPtr depth_img = cv_bridge::CvImage(header, encoding, mono_img).toImageMsg();
			_pub_DepthImage->publish(*depth_img);
			
			// Record depth video if enabled
			if (_record_video) {
				// Convert to 8-bit for video if needed
				cv::Mat video_frame;
				if (mono_img.type() == CV_8UC1) {
					video_frame = mono_img;
				} else {
					mono_img.convertTo(video_frame, CV_8UC1, 255.0 / 65535.0);
				}
				
				if (!_video_writer_depth.isOpened()) {
					// Initialize video writer on first frame
					std::string filename = _video_output_path + "_depth.mp4";
					int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
					cv::Size frame_size = video_frame.size();
					_video_writer_depth.open(filename, fourcc, _video_fps, frame_size, false);  // false = grayscale
					if (_video_writer_depth.isOpened()) {
						RCLCPP_INFO(this->get_logger(), "Recording depth video to: %s (size: %dx%d, fps: %d)", 
							filename.c_str(), frame_size.width, frame_size.height, _video_fps);
					} else {
						RCLCPP_ERROR(this->get_logger(), "Failed to open video writer for: %s", filename.c_str());
					}
				}
				if (_video_writer_depth.isOpened()) {
					_video_writer_depth.write(video_frame);
				}
			}
			
			if (_8bpp) _depth_image = mono_img;
		}
		if (_has_intensity_image && _pub_IntensityImage->get_subscription_count() > 0) {
			cv::Mat mono_img = cv::Mat(_intensity_image.size(), mode);
			
			// Use adjustable percentiles to preserve detail and avoid over-brightening
			cv::Mat non_zero_mask = _intensity_image > 0;
			
			if (cv::countNonZero(non_zero_mask) > 0) {
				// Calculate percentile values for robust normalization
				std::vector<uint16_t> intensity_values;
				intensity_values.reserve(cv::countNonZero(non_zero_mask));
				for (int i = 0; i < _intensity_image.rows; i++) {
					for (int j = 0; j < _intensity_image.cols; j++) {
						uint16_t val = _intensity_image.at<uint16_t>(i, j);
						if (val > 0) {
							intensity_values.push_back(val);
						}
					}
				}
				
				std::sort(intensity_values.begin(), intensity_values.end());
				
				// Use configurable percentiles (default: 0.5th to 99.8th)
				size_t idx_low = intensity_values.size() * _intensity_percentile_low;
				size_t idx_high = intensity_values.size() * _intensity_percentile_high;
				double min_val = intensity_values[idx_low];
				double max_val = intensity_values[idx_high];
				
				// Apply gamma correction to darken and recover texture details
				// Lower gamma = darker image with more visible details in bright areas
				_intensity_image.convertTo(mono_img, mode, max_range / (max_val - min_val), -min_val * max_range / (max_val - min_val));
				mono_img.setTo(0, ~non_zero_mask); // Keep zeros as zero
				
				// Apply gamma correction with configurable gamma value
				if (mode == CV_8UC1) {
					cv::Mat lookup_table(1, 256, CV_8U);
					for (int i = 0; i < 256; i++) {
						lookup_table.at<uint8_t>(i) = cv::saturate_cast<uint8_t>(std::pow(i / 255.0, _intensity_gamma) * 255.0);
					}
					cv::LUT(mono_img, lookup_table, mono_img);
				} else {
					// For 16-bit, apply gamma directly
					mono_img.convertTo(mono_img, CV_32F);
					cv::pow(mono_img / max_range, _intensity_gamma, mono_img);
					mono_img = mono_img * max_range;
					mono_img.convertTo(mono_img, mode);
				}
				
				// Clamp values to valid range
				cv::threshold(mono_img, mono_img, max_range, max_range, cv::THRESH_TRUNC);
				cv::threshold(mono_img, mono_img, 0, 0, cv::THRESH_TOZERO);
			} else {
				mono_img = cv::Mat::zeros(_intensity_image.size(), mode);
			}
			
			cv::resize(mono_img, mono_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, cv::INTER_LINEAR);
			if (_equalize) cv::equalizeHist( mono_img, mono_img );
			if (_flip) cv::flip( mono_img, mono_img, 1);
			sensor_msgs::msg::Image::SharedPtr intensity_img = cv_bridge::CvImage(header, encoding, mono_img).toImageMsg();
			_pub_IntensityImage->publish(*intensity_img);
			
			// Record intensity video if enabled
			if (_record_video) {
				// Convert to 8-bit for video if needed
				cv::Mat video_frame;
				if (mono_img.type() == CV_8UC1) {
					video_frame = mono_img;
				} else {
					mono_img.convertTo(video_frame, CV_8UC1, 255.0 / 65535.0);
				}
				
				if (!_video_writer_intensity.isOpened()) {
					// Initialize video writer on first frame
					std::string filename = _video_output_path + "_intensity.mp4";
					int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
					cv::Size frame_size = video_frame.size();
					_video_writer_intensity.open(filename, fourcc, _video_fps, frame_size, false);  // false = grayscale
					if (_video_writer_intensity.isOpened()) {
						RCLCPP_INFO(this->get_logger(), "Recording intensity video to: %s (size: %dx%d, fps: %d)", 
							filename.c_str(), frame_size.width, frame_size.height, _video_fps);
					} else {
						RCLCPP_ERROR(this->get_logger(), "Failed to open video writer for: %s", filename.c_str());
						_record_video = false;
					}
				}
				if (_video_writer_intensity.isOpened()) {
					_video_writer_intensity.write(video_frame);
				}
			}
			
			if (_8bpp) _intensity_image = mono_img;
		}
		if (_has_reflectance_image && _pub_ReflectanceImage->get_subscription_count() > 0) {
			cv::Mat mono_img = cv::Mat(_reflectance_image.size(), mode);
			cv::normalize(_reflectance_image, mono_img, min_range, max_range, cv::NORM_MINMAX, mode);
			cv::resize(mono_img, mono_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, cv::INTER_LINEAR);
			if (_equalize) cv::equalizeHist( mono_img, mono_img );
			if (_flip) cv::flip( mono_img, mono_img, 1);
			sensor_msgs::msg::Image::SharedPtr reflectance_img = cv_bridge::CvImage(header, encoding, mono_img).toImageMsg();
			_pub_ReflectanceImage->publish(*reflectance_img);
			if (_8bpp) _reflectance_image = mono_img;
		}
		if (_has_noise_image && _pub_NoiseImage->get_subscription_count() > 0) {
			cv::Mat mono_img = cv::Mat(_noise_image.size(), mode);
			cv::normalize(_noise_image, mono_img, min_range, max_range, cv::NORM_MINMAX, mode);
			cv::resize(mono_img, mono_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, cv::INTER_LINEAR);
			if (_equalize) cv::equalizeHist( mono_img, mono_img );
			if (_flip) cv::flip( mono_img, mono_img, 1);
			sensor_msgs::msg::Image::SharedPtr noise_img = cv_bridge::CvImage(header, encoding, mono_img).toImageMsg();
			_pub_NoiseImage->publish(*noise_img);
			if (_8bpp) _noise_image = mono_img;
		}
	} 
	if (_output_mode == ImageOutputMode::GROUP || _output_mode == ImageOutputMode::STACK || _output_mode == ImageOutputMode::ALL) {
		//depth
		cv::Mat mono_img_depth = cv::Mat(_depth_image.size(), mode);
		cv::normalize(_depth_image, mono_img_depth, min_range, max_range, cv::NORM_MINMAX, mode);
		cv::resize(mono_img_depth, mono_img_depth, cv::Size(0,0), _horizontal_scale, _vertical_scale, cv::INTER_LINEAR);
		if (_equalize) cv::equalizeHist( mono_img_depth, mono_img_depth );
		if (_flip) cv::flip( mono_img_depth, mono_img_depth, 1 );
		//intensity
		cv::Mat mono_img_intensity = cv::Mat(_intensity_image.size(), mode);
		cv::normalize(_intensity_image, mono_img_intensity, min_range, max_range, cv::NORM_MINMAX, mode);
		cv::resize(mono_img_intensity, mono_img_intensity, cv::Size(0,0), _horizontal_scale, _vertical_scale, cv::INTER_LINEAR);
		if (_equalize) cv::equalizeHist( mono_img_intensity, mono_img_intensity );
		if (_flip) cv::flip( mono_img_intensity, mono_img_intensity, 1 );
		//reflectance
		cv::Mat mono_img_reflectance = cv::Mat(_reflectance_image.size(), mode);
		cv::normalize(_reflectance_image, mono_img_reflectance, min_range, max_range, cv::NORM_MINMAX, mode);
		cv::resize(mono_img_reflectance, mono_img_reflectance, cv::Size(0,0), _horizontal_scale, _vertical_scale, cv::INTER_LINEAR);
		if (_equalize) cv::equalizeHist( mono_img_reflectance, mono_img_reflectance );
		if (_flip) cv::flip( mono_img_reflectance, mono_img_reflectance, 1 );
		//noise
		cv::Mat mono_img_noise = cv::Mat(_noise_image.size(), mode);
		cv::normalize(_noise_image, mono_img_noise, min_range, max_range, cv::NORM_MINMAX, mode);
		cv::resize(mono_img_noise, mono_img_noise, cv::Size(0,0), _horizontal_scale, _vertical_scale, cv::INTER_LINEAR);
		if (_equalize) cv::equalizeHist( mono_img_noise, mono_img_noise );
		if (_flip) cv::flip( mono_img_noise, mono_img_noise, 1 );

		//count the effective number of images to output
		int img_count = 0;
		img_count += _has_depth_image;
		img_count += _has_intensity_image;
		img_count += _has_reflectance_image;
		img_count += _has_noise_image;

		if (_output_mode == ImageOutputMode::GROUP || _output_mode == ImageOutputMode::ALL) {
			//prerequisite: all above images have same width and height
			auto group_size = mono_img_depth.size();
			group_size.height = group_size.height * img_count; 
			cv::Mat mono_img_group = cv::Mat(group_size, mode);

			auto cols = mono_img_depth.cols;
			auto rows = mono_img_depth.rows;
			img_count = 0;
			if (_has_depth_image) {
				//copy depth image
				mono_img_depth.copyTo(mono_img_group(cv::Rect(0, img_count*rows, cols, rows)));
				img_count++;
			}
			if (_has_intensity_image) {
				//copy intensity image
				mono_img_intensity.copyTo(mono_img_group(cv::Rect(0, img_count*rows, cols, rows)));
				img_count++;
			}
			if (_has_reflectance_image) {
				//copy reflectance image
				mono_img_reflectance.copyTo(mono_img_group(cv::Rect(0, img_count*rows, cols, rows)));
				img_count++;
			}
			if (_has_noise_image) {
				//copy noise image
				mono_img_noise.copyTo(mono_img_group(cv::Rect(0, img_count*rows, cols, rows)));
				img_count++;
			}
			//get a local copy
			_group_image = mono_img_group;

			//publish
			sensor_msgs::msg::Image::SharedPtr output_img = cv_bridge::CvImage(header, encoding, mono_img_group).toImageMsg();
			_pub_GroupImage->publish(*output_img);

		} 
		if (_output_mode == ImageOutputMode::STACK || _output_mode == ImageOutputMode::ALL) {
			//prerequisite: all above images have same width and height
			auto format = CV_16UC1;
			auto encoding = sensor_msgs::image_encodings::MONO16;
			if (_8bpp) {
				format = CV_8UC1;
				encoding = sensor_msgs::image_encodings::MONO8;
			}
			if (img_count == 2) {
				format = CV_16UC3;
				encoding = sensor_msgs::image_encodings::BGR16;
				if (_8bpp) {
					format = CV_8UC3;
					encoding = sensor_msgs::image_encodings::BGR8;
				}
			} else if (img_count == 3) {
				format = CV_16UC3;
				encoding = sensor_msgs::image_encodings::BGR16;
				if (_8bpp) {
					format = CV_8UC3;
					encoding = sensor_msgs::image_encodings::BGR8;
				}
			} else if (img_count == 4) {
				format = CV_16UC4;
				encoding = sensor_msgs::image_encodings::BGRA16;
				if (_8bpp) {
					format = CV_8UC4;
					encoding = sensor_msgs::image_encodings::BGRA8;
				}
			}
			cv::Mat mono_img_stack = cv::Mat(mono_img_depth.size(), format); //n channels

			//special case: if only two images, add space for the 3rd channel
			if (img_count == 2) {
				img_count++;
			}
			std::vector<cv::Mat> images(img_count);
			img_count = 0;

			if (_has_depth_image) {
				images.at(img_count++) = mono_img_depth; 
			}
			if (_has_intensity_image) {
				images.at(img_count++) = mono_img_intensity; 
			}
			if (_has_reflectance_image) {
				images.at(img_count++) = mono_img_reflectance;
			}
			if (_has_noise_image) {
				images.at(img_count++) = mono_img_noise;
			}

			//special case: if only two images, fill with zeros for the 3rd channel
			if (img_count == 2) {
				images.at(img_count) = cv::Mat::zeros(mono_img_depth.rows, mono_img_depth.cols, mode);;	
			}

			//combine all images as separate channels
			cv::merge(images, mono_img_stack);

			//get a local copy
			_stack_image = mono_img_stack;

			//publish
			sensor_msgs::msg::Image::SharedPtr output_img = cv_bridge::CvImage(header, encoding, mono_img_stack).toImageMsg();
			_pub_StackImage->publish(*output_img);
		}
	}

	//output images to PNG files if so requested
	if (_save_images) {
		saveImages();
	}
}

template<class T>
std::string CloudToImage::timeToStr(T ros_t)
{
	(void)ros_t;
	std::stringstream msg;
	const boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_facet *const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S.%f");
	msg.imbue(std::locale(msg.getloc(),f));
	msg << now;
	return msg.str();
}

void CloudToImage::saveImages(const std::string& base_name)
{
	std::string filename;
	if (_output_mode == ImageOutputMode::SINGLE || _output_mode == ImageOutputMode::ALL) {
		if (_has_depth_image) {
			//save the generated depth image
			filename = std::string(base_name + "_depth");
			filename += std::string("_") + timeToStr(this->now()) + std::string(".png");
			if (_8bpp) {
				CloudProjection::cvMatToColorPNG(_depth_image, filename);
			} else {
				CloudProjection::cvMatToDepthPNG(_depth_image, filename);
			}
		}
		if (_has_intensity_image) {
			//save the generated intensity image
			filename = std::string(base_name + "_intensity");
			filename += std::string("_") + timeToStr(this->now()) + std::string(".png");
			if (_8bpp) {
				CloudProjection::cvMatToColorPNG(_intensity_image, filename);
			} else {
				CloudProjection::cvMatToDepthPNG(_intensity_image, filename);
			}
		}

		if (_has_reflectance_image) {
			//save the generated reflectance image
			filename = std::string(base_name + "_reflectance");
			filename += std::string("_") + timeToStr(this->now()) + std::string(".png");
			if (_8bpp) {
				CloudProjection::cvMatToColorPNG(_reflectance_image, filename);
			} else {
				CloudProjection::cvMatToDepthPNG(_reflectance_image, filename);
			}
		}

		if (_has_noise_image) {
			//save the generated noise image
			filename = std::string(base_name + "_noise");
			filename += std::string("_") + timeToStr(this->now()) + std::string(".png");
			if (_8bpp) {
				CloudProjection::cvMatToColorPNG(_noise_image, filename);
			} else {
				CloudProjection::cvMatToDepthPNG(_noise_image, filename);
			}
		}
	} 
	if (_output_mode == ImageOutputMode::GROUP || _output_mode == ImageOutputMode::ALL) {
		//save the generated group image
		filename = std::string(base_name + "_group");
		filename += std::string("_") + timeToStr(this->now()) + std::string(".png");
		CloudProjection::cvMatToColorPNG(_group_image, filename);
	} 
	if (_output_mode == ImageOutputMode::STACK || _output_mode == ImageOutputMode::ALL) {
		//save the generated group image
		filename = std::string(base_name + "_stack");
		filename += std::string("_") + timeToStr(this->now()) + std::string(".png");
		CloudProjection::cvMatToColorPNG(_stack_image, filename);
	}
}

void CloudToImage::fillGaps(cv::Mat& image, int method, const cv::Mat* depth_image)
{
	// Improved gap filling that preserves details while reducing black bars
	// Method 0 (conservative): Only fills 1-2 pixel gaps with strict similarity
	// Method 1 (moderate): Fills up to 3-5 pixel gaps with relaxed similarity  
	// Method 2 (aggressive): Original 5-pass method (may blur details)
	//
	// Key improvements:
	// - Depth-guided filling for intensity images (avoids mixing different surfaces)
	// - Stricter similarity thresholds to preserve edges
	// - Fewer passes to reduce over-smoothing
	// - Horizontal priority (lidar scans are more coherent horizontally)
	
	if (image.empty()) return;
	
	// Method 2: Original aggressive 5-pass method (for backwards compatibility)
	if (method == 2) {
		fillGapsAggressive(image);
		return;
	}
	
	// Conservative (0) and Moderate (1) methods with depth guidance
	int max_gap_size_pass1 = (method == 0) ? 2 : 3;
	int max_gap_size_pass2 = (method == 0) ? 0 : 5;  // Skip pass 2 in conservative mode
	float similarity_threshold_pass1 = (method == 0) ? 0.15f : 0.25f;
	float similarity_threshold_pass2 = 0.40f;
	
	if (image.type() == CV_32FC1) {
		// For float depth images
		
		// Pass 1: Fill small horizontal gaps with strict similarity
		for (int row = 0; row < image.rows; row++) {
			float* row_ptr = image.ptr<float>(row);
			
			int start_valid = -1;
			for (int col = 0; col < image.cols; col++) {
				if (row_ptr[col] > 0.0f) {
					if (start_valid >= 0 && col - start_valid > 1) {
						int gap_size = col - start_valid - 1;
						if (gap_size <= max_gap_size_pass1) {
							float start_val = row_ptr[start_valid];
							float end_val = row_ptr[col];
							float max_val = std::max(start_val, end_val);
							if (max_val > 0.0f && std::abs(end_val - start_val) / max_val < similarity_threshold_pass1) {
								// Linear interpolation
								for (int i = 1; i <= gap_size; i++) {
									float ratio = (float)i / (gap_size + 1);
									row_ptr[start_valid + i] = start_val + ratio * (end_val - start_val);
								}
							}
						}
					}
					start_valid = col;
				}
			}
		}
		
		// Pass 2: Fill medium horizontal gaps (moderate mode only)
		if (max_gap_size_pass2 > 0) {
			for (int row = 0; row < image.rows; row++) {
				float* row_ptr = image.ptr<float>(row);
				
				int start_valid = -1;
				for (int col = 0; col < image.cols; col++) {
					if (row_ptr[col] > 0.0f) {
						if (start_valid >= 0 && col - start_valid > 1) {
							int gap_size = col - start_valid - 1;
							if (gap_size > max_gap_size_pass1 && gap_size <= max_gap_size_pass2) {
								float start_val = row_ptr[start_valid];
								float end_val = row_ptr[col];
								float max_val = std::max(start_val, end_val);
								if (max_val > 0.0f && std::abs(end_val - start_val) / max_val < similarity_threshold_pass2) {
									for (int i = 1; i <= gap_size; i++) {
										float ratio = (float)i / (gap_size + 1);
										row_ptr[start_valid + i] = start_val + ratio * (end_val - start_val);
									}
								}
							}
						}
						start_valid = col;
					}
				}
			}
		}
		
	} else if (image.type() == CV_16UC1) {
		// For uint16 intensity/reflectance/noise images
		
		// Check if we have depth guidance
		bool use_depth_guidance = (depth_image != nullptr && !depth_image->empty() && 
		                            depth_image->type() == CV_32FC1 &&
		                            depth_image->rows == image.rows && 
		                            depth_image->cols == image.cols);
		
		// Pass 1: Fill small horizontal gaps with strict similarity
		for (int row = 0; row < image.rows; row++) {
			uint16_t* row_ptr = image.ptr<uint16_t>(row);
			const float* depth_row = use_depth_guidance ? depth_image->ptr<float>(row) : nullptr;
			
			int start_valid = -1;
			for (int col = 0; col < image.cols; col++) {
				if (row_ptr[col] > 0) {
					if (start_valid >= 0 && col - start_valid > 1) {
						int gap_size = col - start_valid - 1;
						if (gap_size <= max_gap_size_pass1) {
							uint16_t start_val = row_ptr[start_valid];
							uint16_t end_val = row_ptr[col];
							uint16_t max_val = std::max(start_val, end_val);
							
							// Check intensity similarity
							bool intensity_similar = (max_val > 0 && 
							                          std::abs((int)end_val - (int)start_val) * 100 / max_val < (int)(similarity_threshold_pass1 * 100));
							
							// Check depth similarity if available (avoid mixing different surfaces)
							bool depth_similar = true;
							if (use_depth_guidance && depth_row) {
								float start_depth = depth_row[start_valid];
								float end_depth = depth_row[col];
								if (start_depth > 0.0f && end_depth > 0.0f) {
									float max_depth = std::max(start_depth, end_depth);
									// Stricter depth threshold - surfaces must be very similar
									depth_similar = (std::abs(end_depth - start_depth) / max_depth < 0.10f);
								}
							}
							
							if (intensity_similar && depth_similar) {
								// Linear interpolation
								for (int i = 1; i <= gap_size; i++) {
									float ratio = (float)i / (gap_size + 1);
									row_ptr[start_valid + i] = (uint16_t)(start_val + ratio * (end_val - start_val));
								}
							}
						}
					}
					start_valid = col;
				}
			}
		}
		
		// Pass 2: Fill medium horizontal gaps (moderate mode only)
		if (max_gap_size_pass2 > 0) {
			for (int row = 0; row < image.rows; row++) {
				uint16_t* row_ptr = image.ptr<uint16_t>(row);
				const float* depth_row = use_depth_guidance ? depth_image->ptr<float>(row) : nullptr;
				
				int start_valid = -1;
				for (int col = 0; col < image.cols; col++) {
					if (row_ptr[col] > 0) {
						if (start_valid >= 0 && col - start_valid > 1) {
							int gap_size = col - start_valid - 1;
							if (gap_size > max_gap_size_pass1 && gap_size <= max_gap_size_pass2) {
								uint16_t start_val = row_ptr[start_valid];
								uint16_t end_val = row_ptr[col];
								uint16_t max_val = std::max(start_val, end_val);
								
								bool intensity_similar = (max_val > 0 && 
								                          std::abs((int)end_val - (int)start_val) * 100 / max_val < (int)(similarity_threshold_pass2 * 100));
								
								bool depth_similar = true;
								if (use_depth_guidance && depth_row) {
									float start_depth = depth_row[start_valid];
									float end_depth = depth_row[col];
									if (start_depth > 0.0f && end_depth > 0.0f) {
										float max_depth = std::max(start_depth, end_depth);
										depth_similar = (std::abs(end_depth - start_depth) / max_depth < 0.20f);
									}
								}
								
								if (intensity_similar && depth_similar) {
									for (int i = 1; i <= gap_size; i++) {
										float ratio = (float)i / (gap_size + 1);
										row_ptr[start_valid + i] = (uint16_t)(start_val + ratio * (end_val - start_val));
									}
								}
							}
						}
						start_valid = col;
					}
				}
			}
		}
	}
}

void CloudToImage::fillGapsAggressive(cv::Mat& image)
{
	// Original aggressive 5-pass gap filling method
	// This may blur details but fills more gaps
	
	if (image.empty()) return;
	
	if (image.type() == CV_32FC1) {
		// For float depth images
		
		// Pass 1: Fill small horizontal gaps (1-3 pixels) with strict similarity
		for (int row = 0; row < image.rows; row++) {
			float* row_ptr = image.ptr<float>(row);
			
			int start_valid = -1;
			for (int col = 0; col < image.cols; col++) {
				if (row_ptr[col] > 0.0f) {
					if (start_valid >= 0 && col - start_valid > 1) {
						int gap_size = col - start_valid - 1;
						if (gap_size <= 3) {
							float start_val = row_ptr[start_valid];
							float end_val = row_ptr[col];
							if (std::abs(end_val - start_val) / std::max(start_val, end_val) < 0.2f) {
								for (int i = 1; i <= gap_size; i++) {
									float ratio = (float)i / (gap_size + 1);
									row_ptr[start_valid + i] = start_val + ratio * (end_val - start_val);
								}
							}
						}
					}
					start_valid = col;
				}
			}
		}
		
		// Pass 2: Fill medium horizontal gaps (4-8 pixels) with relaxed similarity
		for (int row = 0; row < image.rows; row++) {
			float* row_ptr = image.ptr<float>(row);
			
			int start_valid = -1;
			for (int col = 0; col < image.cols; col++) {
				if (row_ptr[col] > 0.0f) {
					if (start_valid >= 0 && col - start_valid > 1) {
						int gap_size = col - start_valid - 1;
						if (gap_size >= 4 && gap_size <= 8) {
							float start_val = row_ptr[start_valid];
							float end_val = row_ptr[col];
							if (std::abs(end_val - start_val) / std::max(start_val, end_val) < 0.35f) {
								for (int i = 1; i <= gap_size; i++) {
									float ratio = (float)i / (gap_size + 1);
									row_ptr[start_valid + i] = start_val + ratio * (end_val - start_val);
								}
							}
						}
					}
					start_valid = col;
				}
			}
		}
		
		// Pass 3: Vertical interpolation with consistency check
		for (int row = 1; row < image.rows - 1; row++) {
			float* prev_row = image.ptr<float>(row - 1);
			float* curr_row = image.ptr<float>(row);
			float* next_row = image.ptr<float>(row + 1);
			
			for (int col = 1; col < image.cols - 1; col++) {
				if (curr_row[col] == 0.0f) {
					float prev_val = prev_row[col];
					float next_val = next_row[col];
					
					if (prev_val > 0.0f && next_val > 0.0f) {
						if (std::abs(next_val - prev_val) / std::max(prev_val, next_val) < 0.4f) {
							curr_row[col] = (prev_val + next_val) * 0.5f;
						}
					}
				}
			}
		}
		
		// Pass 4: Conservative neighbor average (at least 5 neighbors)
		for (int row = 1; row < image.rows - 1; row++) {
			float* prev_row = image.ptr<float>(row - 1);
			float* curr_row = image.ptr<float>(row);
			float* next_row = image.ptr<float>(row + 1);
			
			for (int col = 1; col < image.cols - 1; col++) {
				if (curr_row[col] == 0.0f) {
					// Collect valid neighbors (8-connected)
					std::vector<float> neighbors;
					neighbors.reserve(8);
					
					if (prev_row[col-1] > 0.0f) neighbors.push_back(prev_row[col-1]);
					if (prev_row[col] > 0.0f) neighbors.push_back(prev_row[col]);
					if (prev_row[col+1] > 0.0f) neighbors.push_back(prev_row[col+1]);
					if (curr_row[col-1] > 0.0f) neighbors.push_back(curr_row[col-1]);
					if (curr_row[col+1] > 0.0f) neighbors.push_back(curr_row[col+1]);
					if (next_row[col-1] > 0.0f) neighbors.push_back(next_row[col-1]);
					if (next_row[col] > 0.0f) neighbors.push_back(next_row[col]);
					if (next_row[col+1] > 0.0f) neighbors.push_back(next_row[col+1]);
					
					// Fill if we have at least 5 neighbors
					if (neighbors.size() >= 5) {
						float sum = 0.0f;
						for (float val : neighbors) {
							sum += val;
						}
						curr_row[col] = sum / neighbors.size();
					}
				}
			}
		}
		
		// Pass 5: Second neighbor pass (at least 4 neighbors)
		for (int row = 1; row < image.rows - 1; row++) {
			float* prev_row = image.ptr<float>(row - 1);
			float* curr_row = image.ptr<float>(row);
			float* next_row = image.ptr<float>(row + 1);
			
			for (int col = 1; col < image.cols - 1; col++) {
				if (curr_row[col] == 0.0f) {
					std::vector<float> neighbors;
					neighbors.reserve(8);
					
					if (prev_row[col-1] > 0.0f) neighbors.push_back(prev_row[col-1]);
					if (prev_row[col] > 0.0f) neighbors.push_back(prev_row[col]);
					if (prev_row[col+1] > 0.0f) neighbors.push_back(prev_row[col+1]);
					if (curr_row[col-1] > 0.0f) neighbors.push_back(curr_row[col-1]);
					if (curr_row[col+1] > 0.0f) neighbors.push_back(curr_row[col+1]);
					if (next_row[col-1] > 0.0f) neighbors.push_back(next_row[col-1]);
					if (next_row[col] > 0.0f) neighbors.push_back(next_row[col]);
					if (next_row[col+1] > 0.0f) neighbors.push_back(next_row[col+1]);
					
					// Fill if we have at least 4 neighbors
					if (neighbors.size() >= 4) {
						float sum = 0.0f;
						for (float val : neighbors) {
							sum += val;
						}
						curr_row[col] = sum / neighbors.size();
					}
				}
			}
		}
		
	} else if (image.type() == CV_16UC1) {
		// For uint16 intensity/reflectance/noise images
		
		// Pass 1: Small horizontal gaps (1-3 pixels)
		for (int row = 0; row < image.rows; row++) {
			uint16_t* row_ptr = image.ptr<uint16_t>(row);
			
			int start_valid = -1;
			for (int col = 0; col < image.cols; col++) {
				if (row_ptr[col] > 0) {
					if (start_valid >= 0 && col - start_valid > 1) {
						int gap_size = col - start_valid - 1;
						if (gap_size <= 3) {
							uint16_t start_val = row_ptr[start_valid];
							uint16_t end_val = row_ptr[col];
							uint16_t max_val = std::max(start_val, end_val);
							if (max_val > 0 && std::abs((int)end_val - (int)start_val) * 100 / max_val < 30) {
								for (int i = 1; i <= gap_size; i++) {
									float ratio = (float)i / (gap_size + 1);
									row_ptr[start_valid + i] = (uint16_t)(start_val + ratio * (end_val - start_val));
								}
							}
						}
					}
					start_valid = col;
				}
			}
		}
		
		// Pass 2: Medium horizontal gaps (4-8 pixels)
		for (int row = 0; row < image.rows; row++) {
			uint16_t* row_ptr = image.ptr<uint16_t>(row);
			
			int start_valid = -1;
			for (int col = 0; col < image.cols; col++) {
				if (row_ptr[col] > 0) {
					if (start_valid >= 0 && col - start_valid > 1) {
						int gap_size = col - start_valid - 1;
						if (gap_size >= 4 && gap_size <= 8) {
							uint16_t start_val = row_ptr[start_valid];
							uint16_t end_val = row_ptr[col];
							uint16_t max_val = std::max(start_val, end_val);
							if (max_val > 0 && std::abs((int)end_val - (int)start_val) * 100 / max_val < 50) {
								for (int i = 1; i <= gap_size; i++) {
									float ratio = (float)i / (gap_size + 1);
									row_ptr[start_valid + i] = (uint16_t)(start_val + ratio * (end_val - start_val));
								}
							}
						}
					}
					start_valid = col;
				}
			}
		}
		
		// Pass 3: Vertical interpolation
		for (int row = 1; row < image.rows - 1; row++) {
			uint16_t* prev_row = image.ptr<uint16_t>(row - 1);
			uint16_t* curr_row = image.ptr<uint16_t>(row);
			uint16_t* next_row = image.ptr<uint16_t>(row + 1);
			
			for (int col = 1; col < image.cols - 1; col++) {
				if (curr_row[col] == 0) {
					uint16_t prev_val = prev_row[col];
					uint16_t next_val = next_row[col];
					
					if (prev_val > 0 && next_val > 0) {
						uint16_t max_val = std::max(prev_val, next_val);
						if (max_val > 0 && std::abs((int)next_val - (int)prev_val) * 100 / max_val < 50) {
							curr_row[col] = (prev_val + next_val) / 2;
						}
					}
				}
			}
		}
		
		// Pass 4: Neighbor average (at least 5 neighbors)
		for (int row = 1; row < image.rows - 1; row++) {
			uint16_t* prev_row = image.ptr<uint16_t>(row - 1);
			uint16_t* curr_row = image.ptr<uint16_t>(row);
			uint16_t* next_row = image.ptr<uint16_t>(row + 1);
			
			for (int col = 1; col < image.cols - 1; col++) {
				if (curr_row[col] == 0) {
					std::vector<uint16_t> neighbors;
					neighbors.reserve(8);
					
					if (prev_row[col-1] > 0) neighbors.push_back(prev_row[col-1]);
					if (prev_row[col] > 0) neighbors.push_back(prev_row[col]);
					if (prev_row[col+1] > 0) neighbors.push_back(prev_row[col+1]);
					if (curr_row[col-1] > 0) neighbors.push_back(curr_row[col-1]);
					if (curr_row[col+1] > 0) neighbors.push_back(curr_row[col+1]);
					if (next_row[col-1] > 0) neighbors.push_back(next_row[col-1]);
					if (next_row[col] > 0) neighbors.push_back(next_row[col]);
					if (next_row[col+1] > 0) neighbors.push_back(next_row[col+1]);
					
					// Fill if we have at least 5 neighbors
					if (neighbors.size() >= 5) {
						int sum = 0;
						for (uint16_t val : neighbors) {
							sum += val;
						}
						curr_row[col] = (uint16_t)(sum / neighbors.size());
					}
				}
			}
		}
		
		// Pass 5: Second neighbor pass (at least 4 neighbors)
		for (int row = 1; row < image.rows - 1; row++) {
			uint16_t* prev_row = image.ptr<uint16_t>(row - 1);
			uint16_t* curr_row = image.ptr<uint16_t>(row);
			uint16_t* next_row = image.ptr<uint16_t>(row + 1);
			
			for (int col = 1; col < image.cols - 1; col++) {
				if (curr_row[col] == 0) {
					std::vector<uint16_t> neighbors;
					neighbors.reserve(8);
					
					if (prev_row[col-1] > 0) neighbors.push_back(prev_row[col-1]);
					if (prev_row[col] > 0) neighbors.push_back(prev_row[col]);
					if (prev_row[col+1] > 0) neighbors.push_back(prev_row[col+1]);
					if (curr_row[col-1] > 0) neighbors.push_back(curr_row[col-1]);
					if (curr_row[col+1] > 0) neighbors.push_back(curr_row[col+1]);
					if (next_row[col-1] > 0) neighbors.push_back(next_row[col-1]);
					if (next_row[col] > 0) neighbors.push_back(next_row[col]);
					if (next_row[col+1] > 0) neighbors.push_back(next_row[col+1]);
					
					// Fill if we have at least 4 neighbors
					if (neighbors.size() >= 4) {
						int sum = 0;
						for (uint16_t val : neighbors) {
							sum += val;
						}
						curr_row[col] = (uint16_t)(sum / neighbors.size());
					}
				}
			}
		}
	}
}

}

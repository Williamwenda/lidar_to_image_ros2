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
	_overlapping(0),
	_output_mode(ImageOutputMode::SINGLE)
{
	_depth_image = cv::Mat::zeros(1, 1, CV_32FC1);
	_intensity_image = cv::Mat::zeros(1, 1, CV_16UC1);
	_reflectance_image = cv::Mat::zeros(1, 1, CV_16UC1);
	_noise_image = cv::Mat::zeros(1, 1, CV_16UC1);	
	_group_image = cv::Mat::zeros(1, 1, CV_16UC1);
	_stack_image = cv::Mat::zeros(1, 1, CV_16UC3);
}

CloudToImage::~CloudToImage() 
{
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

	//Mossman corrections seem to help on Velodyne
	if (boost::iequals(_sensor_model, "HDL-64") || boost::iequals(_sensor_model, "HDL-32") || boost::iequals(_sensor_model, "VLP-16")) {
		_cloud_proj->loadMossmanCorrections();
	}

	//create the subscribers and publishers
	_sub_PointCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		_cloud_topic, 10, 
		std::bind(&CloudToImage::pointCloudCallback, this, std::placeholders::_1));
	
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

void CloudToImage::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
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
	}

	//get a local copy of each image
	_depth_image = _cloud_proj->depth_image();
	_intensity_image = _cloud_proj->intensity_image();
	_reflectance_image = _cloud_proj->reflectance_image();
	_noise_image = _cloud_proj->noise_image();
	
	// Fill gaps in images to remove black bars/artifacts
	if (_fill_gaps) {
		if (_has_depth_image) {
			fillGaps(_depth_image);
		}
		if (_has_intensity_image) {
			fillGaps(_intensity_image);
		}
		if (_has_reflectance_image) {
			fillGaps(_reflectance_image);
		}
		if (_has_noise_image) {
			fillGaps(_noise_image);
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
			if (_8bpp) _depth_image = mono_img;
		}
		if (_has_intensity_image && _pub_IntensityImage->get_subscription_count() > 0) {
			cv::Mat mono_img = cv::Mat(_intensity_image.size(), mode);
			
			// Better normalization: use percentile-based approach to avoid outliers
			// This gives much better contrast for Ouster data
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
				size_t idx_low = intensity_values.size() * 0.02;  // 2nd percentile
				size_t idx_high = intensity_values.size() * 0.98; // 98th percentile
				double min_val = intensity_values[idx_low];
				double max_val = intensity_values[idx_high];
				
				// Normalize using percentile range for better contrast
				_intensity_image.convertTo(mono_img, mode, max_range / (max_val - min_val), -min_val * max_range / (max_val - min_val));
				mono_img.setTo(0, ~non_zero_mask); // Keep zeros as zero
				
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

void CloudToImage::fillGaps(cv::Mat& image, int method)
{
	// Multi-pass gap filling to match Ouster's default image quality
	// Pass 1: Small horizontal gaps with strict similarity
	// Pass 2: Medium horizontal gaps with relaxed similarity
	// Pass 3: Vertical interpolation with consistency check
	// Pass 4: Diagonal/neighbor interpolation for remaining gaps
	
	(void)method;  // Method parameter reserved for future use
	
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
		
		// Pass 4: Conservative neighbor average for remaining isolated gaps only
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
					
					// Only fill if we have at least 5 neighbors AND they are similar (not scattered)
					if (neighbors.size() >= 5) {
						float sum = 0.0f;
						float min_val = neighbors[0];
						float max_val = neighbors[0];
						for (float val : neighbors) {
							sum += val;
							min_val = std::min(min_val, val);
							max_val = std::max(max_val, val);
						}
						
						// Only fill if neighbors are consistent (within 25% range)
						if ((max_val - min_val) / max_val < 0.25f) {
							curr_row[col] = sum / neighbors.size();
						}
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
		
		// Pass 4: Conservative neighbor average for remaining isolated gaps only
		for (int row = 1; row < image.rows - 1; row++) {
			uint16_t* prev_row = image.ptr<uint16_t>(row - 1);
			uint16_t* curr_row = image.ptr<uint16_t>(row);
			uint16_t* next_row = image.ptr<uint16_t>(row + 1);
			
			for (int col = 1; col < image.cols - 1; col++) {
				if (curr_row[col] == 0) {
					// Collect valid neighbors (8-connected)
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
					
					// Only fill if we have at least 5 neighbors AND they are similar
					if (neighbors.size() >= 5) {
						int sum = 0;
						uint16_t min_val = neighbors[0];
						uint16_t max_val = neighbors[0];
						for (uint16_t val : neighbors) {
							sum += val;
							min_val = std::min(min_val, val);
							max_val = std::max(max_val, val);
						}
						
						// Only fill if neighbors are consistent (within 30% range)
						if (max_val > 0 && (max_val - min_val) * 100 / max_val < 30) {
							curr_row[col] = (uint16_t)(sum / neighbors.size());
						}
					}
				}
			}
		}
	}
}

}


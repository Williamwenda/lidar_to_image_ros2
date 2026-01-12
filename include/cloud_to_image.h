
#ifndef CLOUD_TO_IMAGE_H_
#define CLOUD_TO_IMAGE_H_

#include "projection_params.h"
#include "cloud_projection.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>

namespace cloud_to_image
{
class CloudToImage : public rclcpp::Node
{
	public:
		enum class ImageOutputMode { SINGLE, GROUP, STACK, ALL };

		explicit CloudToImage();
		~CloudToImage();

		void init();
		void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input);
		void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
		void publishImages(const std_msgs::msg::Header& header);
		void saveImages(const std::string& base_name = std::string("cloud2image"));

	private:
		//!
		//! \brief getParam Get parameter from node handle
		//! \param param_name Key string
		//! \param default_value Default value if not found
		//! \return The parameter value
		//!
		template <typename T>
		T getParam(const std::string& param_name, T default_value);

		template <class T>
		static void DoNotFree(T*) {}

		template<class T>
		static std::string timeToStr(T ros_t);
		
		//! \brief fillGaps Fill missing data gaps in images using interpolation
		void fillGaps(cv::Mat& image, int method = 0);

	private:
		CloudProjection* _cloud_proj;

		std::string _cloud_topic;
		std::string _sensor_model;
		std::string _point_type;
		std::string _depth_image_topic;
		std::string _intensity_image_topic;
		std::string _reflectance_image_topic;
		std::string _noise_image_topic;

		bool _has_depth_image;
		bool _has_intensity_image;
		bool _has_reflectance_image;
		bool _has_noise_image;

		bool _save_images;
		bool _8bpp;
		bool _equalize;
		bool _flip;
		bool _fill_gaps;  // Fill missing data gaps
		
		// Video recording
		bool _record_video;
		std::string _video_output_path;
		cv::VideoWriter _video_writer_depth;
		cv::VideoWriter _video_writer_intensity;
		int _video_fps;
		
		// Intensity visualization parameters
		double _intensity_gamma;
		double _intensity_percentile_low;
		double _intensity_percentile_high;

		float _horizontal_scale;
		float _vertical_scale;

		unsigned int _overlapping;

		ImageOutputMode _output_mode;

		cv::Mat _depth_image;
		cv::Mat _intensity_image;
		cv::Mat _reflectance_image;
		cv::Mat _noise_image;
		cv::Mat _group_image;
		cv::Mat _stack_image;
		
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_PointCloud;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub_Odometry;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_DepthImage;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_IntensityImage;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_ReflectanceImage;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_NoiseImage;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_GroupImage;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_StackImage;
		
		// Motion compensation data
		bool _motion_compensation;
		std::string _odom_topic;
		Eigen::Vector3f _latest_linear_velocity;
		Eigen::Vector3f _latest_angular_velocity;
		bool _has_odom_data;
};
}

#endif // CLOUD_TO_IMAGE_H_

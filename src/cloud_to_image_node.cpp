#include "cloud_to_image.h"
#include <rclcpp/rclcpp.hpp>

using namespace cloud_to_image;

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	
	auto node = std::make_shared<CloudToImage>();
	node->init();
	
	rclcpp::spin(node);
	rclcpp::shutdown();
	
	return 0;
}

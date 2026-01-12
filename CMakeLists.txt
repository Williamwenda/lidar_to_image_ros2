cmake_minimum_required(VERSION 3.8)
project(lidar_to_image_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Cloud projection library
add_library(cloud_projection SHARED
  include/angles.h
  include/sensor_params.h
  src/sensor_params.cpp
  include/projection_params.h
  src/projection_params.cpp
  include/pcl_point_types.h
  include/cloud_projection.h
  src/cloud_projection.cpp
)

ament_target_dependencies(cloud_projection
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  cv_bridge
  image_transport
)

target_link_libraries(cloud_projection
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  yaml-cpp
)

# Cloud2image node
add_executable(cloud2image
  src/cloud_to_image_node.cpp
  include/cloud_to_image.h
  src/cloud_to_image.cpp
)

ament_target_dependencies(cloud2image
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  pcl_conversions
  cv_bridge
  image_transport
)

target_link_libraries(cloud2image
  cloud_projection
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

# Install targets
install(TARGETS
  cloud_projection
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  cloud2image
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Uncomment the following line if you want default linters
  # ament_lint_auto_find_test_dependencies()
  
  # Add tests here if needed
endif()

ament_package()

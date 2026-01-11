"""Launch file for LiDAR Cloud to Image ROS2 node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for cloud2image node."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('lidar_to_image_ros2')
    
    # Declare launch arguments
    proj_params_arg = DeclareLaunchArgument(
        'proj_params',
        default_value=os.path.join(pkg_dir, 'config', 'projection_params.yaml'),
        description='Path to the sensor projection parameters file'
    )
    
    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/points_raw',
        description='LiDAR pointcloud topic name'
    )
    
    sensor_model_arg = DeclareLaunchArgument(
        'sensor_model',
        default_value='OS-1-64-1024-HIGH',
        description='Model of the LiDAR sensor for this conversion (use *-HIGH variants for taller images matching Ouster default)'
    )
    
    point_type_arg = DeclareLaunchArgument(
        'point_type',
        default_value='XYZI',
        description='Point format of the 3D pointcloud, valid options: XYZ, XYZI, XYZIR, XYZIF, XYZIFN'
    )
    
    depth_image_topic_arg = DeclareLaunchArgument(
        'depth_image_topic',
        default_value='/c2i_depth_image',
        description='Topic name for the depth (range) output image'
    )
    
    intensity_image_topic_arg = DeclareLaunchArgument(
        'intensity_image_topic',
        default_value='/c2i_intensity_image',
        description='Topic name for the intensity output image'
    )
    
    reflectance_image_topic_arg = DeclareLaunchArgument(
        'reflectance_image_topic',
        default_value='/c2i_reflectance_image',
        description='Topic name for the reflectance output image'
    )
    
    noise_image_topic_arg = DeclareLaunchArgument(
        'noise_image_topic',
        default_value='/c2i_noise_image',
        description='Topic name for the noise output image'
    )
    
    h_scale_arg = DeclareLaunchArgument(
        'h_scale',
        default_value='1.0',
        description='Horizontal scale factor'
    )
    
    v_scale_arg = DeclareLaunchArgument(
        'v_scale',
        default_value='1.0',
        description='Vertical scale factor'
    )
    
    output_mode_arg = DeclareLaunchArgument(
        'output_mode',
        default_value='SINGLE',
        description='How output images are published. SINGLE: each image is independent, GROUP: all images are grouped into one, STACK: all images are combined one per channel, ALL: single + group + stack'
    )
    
    save_images_arg = DeclareLaunchArgument(
        'save_images',
        default_value='false',
        description='Output also the PNG files for the generated images'
    )
    
    overlapping_arg = DeclareLaunchArgument(
        'overlapping',
        default_value='0',
        description='Whether to copy a block of image from left to the right to close the circle'
    )
    
    eight_bpp_arg = DeclareLaunchArgument(
        '8bpp',
        default_value='false',
        description='Whether to output 8 bit mode images'
    )
    
    equalize_arg = DeclareLaunchArgument(
        'equalize',
        default_value='false',
        description='Whether to equalize histogram for output images'
    )
    
    flip_arg = DeclareLaunchArgument(
        'flip',
        default_value='false',
        description='Whether to flip horizontally the image'
    )
    
    # Create node
    cloud2image_node = Node(
        package='lidar_to_image_ros2',
        executable='cloud2image',
        name='cloud2image',
        output='screen',
        parameters=[{
            'proj_params': LaunchConfiguration('proj_params'),
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'sensor_model': LaunchConfiguration('sensor_model'),
            'point_type': LaunchConfiguration('point_type'),
            'depth_image_topic': LaunchConfiguration('depth_image_topic'),
            'intensity_image_topic': LaunchConfiguration('intensity_image_topic'),
            'reflectance_image_topic': LaunchConfiguration('reflectance_image_topic'),
            'noise_image_topic': LaunchConfiguration('noise_image_topic'),
            'h_scale': LaunchConfiguration('h_scale'),
            'v_scale': LaunchConfiguration('v_scale'),
            'output_mode': LaunchConfiguration('output_mode'),
            'save_images': LaunchConfiguration('save_images'),
            'overlapping': LaunchConfiguration('overlapping'),
            '8bpp': LaunchConfiguration('8bpp'),
            'equalize': LaunchConfiguration('equalize'),
            'flip': LaunchConfiguration('flip'),
        }]
    )
    
    return LaunchDescription([
        proj_params_arg,
        cloud_topic_arg,
        sensor_model_arg,
        point_type_arg,
        depth_image_topic_arg,
        intensity_image_topic_arg,
        reflectance_image_topic_arg,
        noise_image_topic_arg,
        h_scale_arg,
        v_scale_arg,
        output_mode_arg,
        save_images_arg,
        overlapping_arg,
        eight_bpp_arg,
        equalize_arg,
        flip_arg,
        cloud2image_node
    ])

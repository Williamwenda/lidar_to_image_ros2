"""Launch file to play ROS2 bag and convert Ouster point cloud to images."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for playing bag and converting to images."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('lidar_to_image_ros2')
    
    # Declare launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/wenda/ASRL/vtr3/data/ouster_data/rosbag2_2025_04_27-18_34_00_0',
        description='Path to the ROS2 bag directory'
    )
    
    play_rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate multiplier'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='Loop the bag playback'
    )
    
    sensor_model_arg = DeclareLaunchArgument(
        'sensor_model',
        default_value='OS-1-64-1024-HIGH',
        description='Ouster sensor model (OS-1-64-1024-HIGH for 128-row output, OS-1-64-1024 for 64-row, etc.)'
    )
    
    point_type_arg = DeclareLaunchArgument(
        'point_type',
        default_value='XYZIR',
        description='Point cloud type (XYZI, XYZIR, XYZIF, XYZIFN) - Use XYZIR for Ouster to leverage ring field'
    )
    
    output_mode_arg = DeclareLaunchArgument(
        'output_mode',
        default_value='SINGLE',
        description='Output mode: SINGLE, GROUP, STACK, or ALL'
    )
    
    fill_gaps_arg = DeclareLaunchArgument(
        'fill_gaps',
        default_value='true',
        description='Enable gap filling to remove black horizontal bars'
    )
    
    # ROS2 bag play command
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', 
             LaunchConfiguration('bag_path'),
             '--rate', LaunchConfiguration('rate'),
             '--loop'] if LaunchConfiguration('loop') == 'true' else 
            ['ros2', 'bag', 'play', 
             LaunchConfiguration('bag_path'),
             '--rate', LaunchConfiguration('rate')],
        output='screen',
        name='bag_play'
    )
    
    # Cloud to image node
    cloud2image_node = Node(
        package='lidar_to_image_ros2',
        executable='cloud2image',
        name='cloud2image',
        output='screen',
        parameters=[{
            'proj_params': os.path.join(pkg_dir, 'config', 'projection_params.yaml'),
            'cloud_topic': '/ouster/points',
            'sensor_model': LaunchConfiguration('sensor_model'),
            'point_type': LaunchConfiguration('point_type'),
            'depth_image_topic': '/c2i_depth_image',
            'intensity_image_topic': '/c2i_intensity_image',
            'reflectance_image_topic': '/c2i_reflectance_image',
            'noise_image_topic': '/c2i_noise_image',
            'h_scale': 1.0,
            'v_scale': 1.0,
            'output_mode': LaunchConfiguration('output_mode'),
            'save_images': False,
            'overlapping': 0,
            '8bpp': True,
            'equalize': False,
            'flip': False,
            'fill_gaps': LaunchConfiguration('fill_gaps'),
        }]
    )
    
    return LaunchDescription([
        bag_path_arg,
        play_rate_arg,
        loop_arg,
        sensor_model_arg,
        point_type_arg,
        output_mode_arg,
        fill_gaps_arg,
        bag_play,
        cloud2image_node
    ])

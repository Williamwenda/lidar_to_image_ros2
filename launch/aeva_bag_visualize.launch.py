"""Launch file to play Aeva lidar ROS2 bag and convert point cloud to images."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for playing Aeva bag and converting to images."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('lidar_to_image_ros2')
    
    # Declare launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/wenda/ASRL/vtr3/data/lidar_intensity/rosbag2_2026_01_12-01_51_48',
        description='Path to the Aeva ROS2 bag directory'
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
        default_value='AEVA-AERIES-II',
        description='Aeva sensor model configuration'
    )
    
    point_type_arg = DeclareLaunchArgument(
        'point_type',
        default_value='XYZIT',
        description='Point cloud type - XYZIT for Aeva with motion compensation'
    )
    
    motion_compensation_arg = DeclareLaunchArgument(
        'motion_compensation',
        default_value='true',
        description='Enable motion compensation using odometry data'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/vtr/odometry',
        description='Odometry topic for motion compensation'
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
    
    record_video_arg = DeclareLaunchArgument(
        'record_video',
        default_value='false',
        description='Enable video recording of depth and intensity images'
    )
    
    video_output_path_arg = DeclareLaunchArgument(
        'video_output_path',
        default_value='./aeva_lidar_video',
        description='Output path prefix for video files (without extension)'
    )
    
    video_fps_arg = DeclareLaunchArgument(
        'video_fps',
        default_value='10',
        description='Video recording frame rate (fps)'
    )
    
    intensity_gamma_arg = DeclareLaunchArgument(
        'intensity_gamma',
        default_value='0.6',
        description='Gamma correction for intensity (0.5=darker/more detail, 0.6=balanced, 1.0=linear)'
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
            'cloud_topic': '/aeva/sensor/point_cloud',
            'sensor_model': LaunchConfiguration('sensor_model'),
            'point_type': LaunchConfiguration('point_type'),
            'depth_image_topic': '/aeva/c2i_depth_image',
            'intensity_image_topic': '/aeva/c2i_intensity_image',
            'reflectance_image_topic': '/aeva/c2i_reflectance_image',
            'noise_image_topic': '/aeva/c2i_noise_image',
            'h_scale': 1.0,
            'v_scale': 1.0,
            'output_mode': LaunchConfiguration('output_mode'),
            'save_images': False,
            'overlapping': 0,
            '8bpp': True,
            'equalize': False,
            'flip': True,
            'fill_gaps': LaunchConfiguration('fill_gaps'),
            'motion_compensation': LaunchConfiguration('motion_compensation'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'record_video': LaunchConfiguration('record_video'),
            'video_output_path': LaunchConfiguration('video_output_path'),
            'video_fps': LaunchConfiguration('video_fps'),
            'intensity_gamma': LaunchConfiguration('intensity_gamma'),
            'intensity_percentile_low': 0.005,
            'intensity_percentile_high': 0.998,
        }]
    )
    
    return LaunchDescription([
        bag_path_arg,
        play_rate_arg,
        loop_arg,
        sensor_model_arg,
        point_type_arg,
        motion_compensation_arg,
        odom_topic_arg,
        output_mode_arg,
        fill_gaps_arg,
        record_video_arg,
        video_output_path_arg,
        video_fps_arg,
        intensity_gamma_arg,
        bag_play,
        cloud2image_node
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    carla_gnss_processing_node = Node(
        package='carla_gnss_processing',
        executable='carla_gnss_processing_node',
        name='carla_gnss_processing_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'main_topic': '/carla/hero/gnss_main',
            'aux_topic': '/carla/hero/gnss_aux',
            'gnss_topic': '/carla/hero/gnss',
            # Distance between the two GNSS sensors in the CARLA sensor config
            'baseline_length': 2.0,
            'baseline_tolerance': 0.5,
            # Where the CARLA map origin is placed on Earth
            'origin_lat': 45.0,
            'origin_lon': 9.0
        }]
    )

    return LaunchDescription([
        carla_gnss_processing_node
    ])

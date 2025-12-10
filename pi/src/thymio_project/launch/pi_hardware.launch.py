import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # LIDAR Driver: https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2
    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD06',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD06'},
            {'topic_name': 'scan'},
            {'frame_id': 'base_laser'},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False}
        ]
    )

    #TF - attach LIDAR to thymio
    # (x,y,z) compared to base_link (center of robot)
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.12', '0.0', '0.0', '0.0', 'base_link', 'base_laser']
    )


    return LaunchDescription([
        base_to_laser_tf,
        ldlidar_node
    ])

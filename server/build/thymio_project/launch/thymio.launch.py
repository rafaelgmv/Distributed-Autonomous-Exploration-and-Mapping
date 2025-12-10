import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # --- 1. CONFIGURAÇÕES ---
    # Caminho para o teu ficheiro YAML que consertou o SLAM
    #slam_config_path = '/home/rafael/ldlidar_ros2_ws/src/ldlidar_stl_ros2/slam_config.yaml'
    config_dir = os.path.join(get_package_share_directory('thymio_project'), 'config')
    slam_config_path = os.path.join(config_dir, 'slam_config.yaml')
    
    # --- 2. LIDAR ---
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
            {'enable_angle_crop_func': False},
        ]
    )

    # link between base_link and base_laser
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.02675', '-0.0025', '0.05', '0', '0', '0', 'base_link', 'base_laser']
    )
    
    # SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': slam_config_path}.items()
    )

    # api and logic
    main_node = Node(
        package='thymio_project',
        executable='main',
        name='thymio_main',
        output='screen'
    )


    return LaunchDescription([
        base_to_laser_tf,
        ldlidar_node,
        slam_launch,
        main_node
    ])

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_thymio = get_package_share_directory('thymio_project')
    
    # 1. SLAM TOOLBOX
    slam_config_path = os.path.join(pkg_thymio, 'config', 'slam_config.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': slam_config_path}.items()
    )

    # 2. THYMIO BRAIN (Agora corre no PC, usando o Dongle do PC)
    thymio_driver = Node(
        package='thymio_project',
        executable='main',
        name='thymio_driver',
        output='screen'
    )

    # 3. RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    return LaunchDescription([
        slam_launch,
        thymio_driver, # <--- Adicionado aqui
        rviz_node
    ])
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('odrive_mit_example')

    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    declare_use_mock_hardware = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Start robot with mock hardware mirroring command to its states.'
    )

    # Include mit_example.launch.py
    mit_example = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'mit_example.launch.py')
        ),
        launch_arguments={'use_mock_hardware': use_mock_hardware}.items()
    )

    # IK Node
    ik_node = Node(
        package='odrive_mit_example',
        executable='simple_ik_node.py',
        name='simple_ik_node',
        output='screen'
    )

    return LaunchDescription([
        declare_use_mock_hardware,
        mit_example,
        ik_node
    ])

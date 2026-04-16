import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_file = os.path.join(pkg_dir, 'gim_test.urdf')
    controllers_file = os.path.join(pkg_dir, 'gim_controllers.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file,
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
             'joint_state_broadcaster'],
        output='screen',
    )

    leg_impedance_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
             'leg_impedance_controller'],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        leg_impedance_controller_spawner,
    ])

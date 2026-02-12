from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch argument
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    declare_use_mock_hardware = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Start robot with mock hardware mirroring command to its states.'
    )

    # 1. Get the path to your URDF and Config
    pkg_path = get_package_share_directory('odrive_mit_example')
    xacro_file = os.path.join(pkg_path, 'description', 'urdf', 'clue_right_leg.urdf.xacro')
    
    # 2. Process the Xacro file (convert to XML)
    # We delay processing until runtime to use the LaunchConfiguration, but standard xacro processing
    # in launch files typically happens before Node definition if we use Python xacro bindings.
    # To pass LaunchConfiguration to xacro, we usually need to use Command substitution or process conditionally.
    # For simplicity here, we can re-process based on default or passed args if we weren't using LaunchConfiguration directly in Python.
    # However, xacro.process_file doesn't accept LaunchConfiguration objects directly.
    # We will use Command substitution which is the standard way to handle xacro with launch args in ROS 2.
    
    from launch.substitutions import Command, PathJoinSubstitution
    
    robot_description_content = Command([
        'xacro ', xacro_file, 
        ' use_mock_hardware:=', use_mock_hardware
    ])
    robot_description = {'robot_description': robot_description_content}

    # 3. Path to the controller config file
    controller_config = os.path.join(
        get_package_share_directory('odrive_mit_example'),
        'config',
        'mit_controllers.yaml'
    )

    # Actuator config
    actuator_config = os.path.join(
        get_package_share_directory('odrive_mit_example'),
        'config',
        'actuator_config.yaml'
    )

    # 4. Define the Controller Manager Node (The Brain)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, actuator_config],
        output="screen",
    )

    # 5. Define the Joint State Broadcaster Spawner
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # 6. Define Your Custom Impedance Controller Spawner
    leg_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_impedance_controller"],
    )

    # Ensure the leg controller only starts AFTER the broadcaster is ready
    delayed_leg_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[leg_controller],
        )
    )

    return LaunchDescription([
        declare_use_mock_hardware,
        control_node,
        joint_state_broadcaster,
        delayed_leg_controller,
    ])
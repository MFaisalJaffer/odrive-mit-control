from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    xacro_file = os.path.join(pkg_path, 'description', 'urdf', 'mit_robot.urdf.xacro')
    
    # 2. Process the Xacro file
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

    # 4. Define the Controller Manager Node (The Brain)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
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

    # 7. Robot State Publisher (Required for Foxglove/RViz visualization)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 8. Foxglove Bridge
    # Check if foxglove_bridge is installed or in workspace
    # We will assume it is available as 'foxglove_bridge' package
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'send_buffer_limit': 10000000}], # Optional parameter
    )

    return LaunchDescription([
        declare_use_mock_hardware,
        control_node,
        joint_state_broadcaster,
        delayed_leg_controller,
        robot_state_publisher,
        foxglove_bridge
    ])


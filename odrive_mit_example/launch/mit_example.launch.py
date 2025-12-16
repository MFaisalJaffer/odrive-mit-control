import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("odrive_mit_example"), "description", "urdf", "mit_robot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("odrive_mit_example"),
            "config",
            "mit_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawn MIT controllers
    # To use MIT control, you must activate the controllers for the interfaces you want to use.
    # Typically, you would activate all of them to have full control over the MIT command.
    
    controllers = [
        "mit_position_controller",
        "mit_velocity_controller",
        "mit_effort_controller",
        "mit_kp_controller",
        "mit_kd_controller",
    ]

    spawner_nodes = []
    for controller in controllers:
        spawner_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager"],
            )
        )

    # Delay start of robot_controllers after `joint_state_broadcaster`
    # We chain them so they start one after another (or all after JSB)
    delay_spawners = []
    for node in spawner_nodes:
        delay_spawners.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[node],
                )
            )
        )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
    ] + delay_spawners

    return LaunchDescription(nodes)


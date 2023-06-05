from inspect import Parameter
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_prefix = get_package_share_directory('miyalab_manual_robot_controller')
   
    container = Node(
	    package='rclcpp_components',
		executable='component_container',
		name = 'manual_robot_controller_container',
		emulate_tty = True,
		output = 'screen'
	)

    components = LoadComposableNodes(
        target_container="manual_robot_controller_container",
        composable_node_descriptions=[
            ComposableNode(
                package='miyalab_manual_robot_controller',
                plugin='MiYALAB::ROS2::ManualRobotController2D',
                name='joystick',
                parameters=[join(pkg_prefix, "cfg/controller_2d.yaml")],
                remappings=[
                    # publisher
                    ("~/state",        "~/state"),
                    ("~/is_connected", "~/is_connected")
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]

    )
    return LaunchDescription([
        container,
        components
    ])
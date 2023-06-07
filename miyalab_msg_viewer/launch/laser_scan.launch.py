from inspect import Parameter
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_prefix = get_package_share_directory('miyalab_msg_viewer')
   
    container = Node(
	    package='rclcpp_components',
		executable='component_container',
		name = 'viewer_container',
		emulate_tty = True,
		output = 'screen'
	)

    components = LoadComposableNodes(
        target_container="viewer_container",
        composable_node_descriptions=[
            ComposableNode(
                package='miyalab_msg_viewer',
                plugin='MiYALAB::ROS2::LaserScanViewer',
                name='viewer',
                parameters=[join(pkg_prefix, "cfg/laser_scan.yaml")],
                remappings=[
                    # subscriber
                    ("/laser/scan", "/a2m8/scan"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]

    )
    return LaunchDescription([
        container,
        components
    ])
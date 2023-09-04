from inspect import Parameter
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_prefix = get_package_share_directory('miyalab_camera')
   
    container = Node(
	    package='rclcpp_components',
		executable='component_container',
		name = 'camera_container',
		emulate_tty = True,
		output = 'screen'
	)

    components = LoadComposableNodes(
        target_container="camera_container",
        composable_node_descriptions=[
            ComposableNode(
                package='miyalab_camera',
                plugin='MiYALAB::ROS2::Camera',
                name='camera',
                parameters=[join(pkg_prefix, "cfg/camera.yaml")],
                remappings=[
                    # publisher
                    ("~/image", "~/image"),

                    # service
                    ("~/get_camera_parameter, ~/get_camera_parameter"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]

    )
    return LaunchDescription([
        container,
        components
    ])

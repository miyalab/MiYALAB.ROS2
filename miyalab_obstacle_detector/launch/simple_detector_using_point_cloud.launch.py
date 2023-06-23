from inspect import Parameter
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_prefix = get_package_share_directory('miyalab_obstacle_detector')
    components = LoadComposableNodes(
        target_container="rs_container",
        composable_node_descriptions=[
            ComposableNode(
                package='miyalab_obstacle_detector',
                plugin='MiYALAB::ROS2::SimpleObstacleDetectorUsingPointCloud',
                name='obstacle_detector',
                parameters=[join(pkg_prefix, "cfg/simple_detector_using_point_cloud.yaml")],
                remappings=[
                    # subscriber
                    ("/lidar/points",                    "/rfans16/points_near"),

                    # publisher
                    ("~/obstacles", "~/obstacle_infos"),
                    ("~/image"    , "~/obstacle_image"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ]
    )
    return LaunchDescription([
        components
    ])
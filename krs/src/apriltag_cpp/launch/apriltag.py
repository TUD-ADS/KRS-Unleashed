from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='apriltag_cpp_nodes',
            namespace='apriltag',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='apriltag_cpp',
                    plugin='ApriltagDecimatorNode',
                    name='apriltag_decimate',
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='apriltag_cpp',
                    plugin='ApriltagBlurNode',
                    name='apriltag_blur',
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='apriltag_cpp',
                    plugin='ApriltagThresholdNode',
                    name='apriltag_threshold',
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='apriltag_cpp',
                    plugin='ApriltagContoursNode',
                    name='apriltag_contours',
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='apriltag_cpp',
                    plugin='ApriltagHullsNode',
                    name='apriltag_hulls',
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='apriltag_cpp',
                    plugin='ApriltagQuadsNode',
                    name='apriltag_quads',
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='apriltag_cpp',
                    plugin='ApriltagTagDecodeNode',
                    name='apriltag_tag_decode',
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ]
        )
    ]
    )

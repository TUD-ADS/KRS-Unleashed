from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS


def generate_launch_description():
    intra_process = False
    return LaunchDescription([
        Trace(
            session_name='remote_full_system1',
            events_ust=[
                "ros2_kernels:*",
            ]
            + DEFAULT_EVENTS_ROS,
        ),
        ComposableNodeContainer(
            name='apriltag_cpp_remote',
            namespace='apriltag',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='apriltag_cpp',
                    plugin='apriltag_cpp::ApriltagDecimatorComponent',
                    name='apriltag_decimate',
                    extra_arguments=[
                        {'use_intra_process_comms': intra_process}],
                ),
                # ComposableNode(
                #     package='apriltag_cpp',
                #     plugin='apriltag_cpp::ApriltagBlurComponent',
                #     name='apriltag_blur',
                #     extra_arguments=[
                #         {'use_intra_process_comms': intra_process}],
                # ),
                ComposableNode(
                    package='apriltag_cpp',
                    plugin='apriltag_cpp::ApriltagContoursComponent',
                    name='apriltag_contours',
                    extra_arguments=[
                        {'use_intra_process_comms': intra_process}],
                ),
                ComposableNode(
                    package='apriltag_cpp',
                    plugin='apriltag_cpp::ApriltagHullsComponent',
                    name='apriltag_hulls',
                    extra_arguments=[
                        {'use_intra_process_comms': intra_process}],
                ),
                ComposableNode(
                    package='apriltag_cpp',
                    plugin='apriltag_cpp::ApriltagQuadsComponent',
                    name='apriltag_quads',
                    extra_arguments=[
                        {'use_intra_process_comms': intra_process}],
                ),
                ComposableNode(
                    package='apriltag_cpp',
                    plugin='apriltag_cpp::ApriltagTagDecodeComponent',
                    name='apriltag_tag_decode',
                    extra_arguments=[
                        {'use_intra_process_comms': intra_process}]
                )
            ]
        )]
    )

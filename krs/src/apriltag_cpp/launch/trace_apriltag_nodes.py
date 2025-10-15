"""Launch file for the Trace action."""

from launch import LaunchDescription
from launch_ros.actions import Node
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS


def generate_launch_description():
    return LaunchDescription([
        Trace(
            session_name='apriltag_cpp_nodes_multi',
            events_ust=[
                "ros2_kernels:*"
            ]
            + DEFAULT_EVENTS_ROS,
        ),
        Node(
            package='apriltag_cpp',
            executable='apriltag_decimate_node',
        ),
        Node(
            package='apriltag_cpp',
            executable='apriltag_blur_node',
        ),
        Node(
            package='apriltag_cpp',
            executable='apriltag_threshold_node',
        ),
        Node(
            package='apriltag_cpp',
            executable='apriltag_contours_node',
        ),
        Node(
            package='apriltag_cpp',
            executable='apriltag_hulls_node',
        ),
        Node(
            package='apriltag_cpp',
            executable='apriltag_quads_node',
        ),
        Node(
            package='apriltag_cpp',
            executable='apriltag_tag_decode_node',
        )
    ])

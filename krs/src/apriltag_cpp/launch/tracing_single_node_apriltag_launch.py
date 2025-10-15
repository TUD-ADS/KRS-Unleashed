from launch import LaunchDescription
from launch_ros.actions import Node
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS


def generate_launch_description():
    return LaunchDescription([
        Trace(
            session_name='apriltag_cpp_single',
            events_ust=[
                "ros2_kernels:*",
            ]
            + DEFAULT_EVENTS_ROS,
        ),
        Node(
            package='apriltag_cpp',
            namespace='apriltag',
            executable='apriltag_cpp_node',
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS


def generate_launch_description():
    intra_process = False
    return LaunchDescription([
        Trace(
            session_name='full_system1',
            events_ust=[
                "ros2_kernels:*",
            ]
            + DEFAULT_EVENTS_ROS,
        ),
        ComposableNodeContainer(
            name='apriltag_cpp_nodes',
            namespace='apriltag',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='apriltag_accel',
                    plugin='apriltag_accel::ThresholdFPGA',
                    name='apriltag_threshold_fpga',
                    extra_arguments=[
                        {'use_intra_process_comms': intra_process}],
                )
            ]
        )]
    )

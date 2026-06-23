import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for Mars rover status monitoring mission."""
    container = ComposableNodeContainer(
            name='rover_mission_control',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='rover_components',
                    plugin='rover_components::RobotStatusService',
                    name='robot_status_service'),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
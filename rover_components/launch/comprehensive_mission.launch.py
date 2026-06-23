import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for comprehensive Mars rover mission."""
    
    # Define the comprehensive mission container with both components
    container = ComposableNodeContainer(
            name='mars_rover_mission_control',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Robot Status Monitoring Service
                ComposableNode(
                    package='rover_components',
                    plugin='rover_components::RobotStatusService',
                    name='robot_status_service',
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                # Text Recognition Service for Supply Box Identification
                ComposableNode(
                    package='rover_components',
                    plugin='rover_components::TextRecognitionService',
                    name='text_recognition_service',
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([
        container
    ])
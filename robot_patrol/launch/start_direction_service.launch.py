from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
        
    return LaunchDescription([
        launch_ros.actions.Node(
             package='robot_patrol',
            executable='direction_service_node', output='screen'),
        
    ])
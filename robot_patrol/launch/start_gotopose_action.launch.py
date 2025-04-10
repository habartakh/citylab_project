from launch import LaunchDescription
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


# launch rviz2 node from https://github.com/ros2/rviz/issues/627
def generate_launch_description():
    base_path = os.path.realpath(get_package_share_directory('robot_patrol')) 
    #print(base_path)
    rviz_path=base_path+'/config/config.rviz'
    #print(rviz_path)
    
    return LaunchDescription([
        launch_ros.actions.Node(
             package='robot_patrol',
            executable='go_to_pose_action_node', output='screen'),
        # launch_ros.actions.Node(
        #     package='rviz2',
        #     executable='rviz2',output='screen', arguments=['-d', str(rviz_path)]
        # ),
    ])
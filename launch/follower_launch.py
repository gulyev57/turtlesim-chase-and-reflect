from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='hw_9',
            executable='robot2_leader',
            name='robot2_leader_node',
			output = 'screen'
        ),
        Node(
            package='hw_9',
            executable='robot1_follower_pure_pursuit',
            name='robot1_follower_node',
			output = 'screen'
        )
    ])

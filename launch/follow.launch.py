from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package="turtlebot_bringup",
            node_executable="turtlebot2",
            output="screen"
        ),

        launch_ros.actions.Node(
            package="ydlidar",
            node_executable="ydlidar_node",
            output="screen"
        ),

        launch_ros.actions.Node(
            package="follow_me",
            node_executable="follow_me",
            output="screen"
        ),
    ])

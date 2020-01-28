from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='blinkt_test',
            node_executable='rainbow',
            output='log',
            parameters=[{'brightness' : 0.5}]
        )
    ])

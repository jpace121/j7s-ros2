from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='j7s',
            node_executable='j7s-pub',
            output='log',
            parameters=[{
                'freq' : 100.0,
                'led_index': 1,
                'color': 'aqua',
                'pub_freq':10.0
            }],
            remappings = [
                ('led_state', 'j7s_led_state')
            ]
        )
    ])

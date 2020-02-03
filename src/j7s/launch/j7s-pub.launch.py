from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='j7s',
            node_executable='j7s-pub',
            node_name='aqua_pub',
            output='log',
            parameters=[{
                'freq': 0.5,
                'led_index': 0,
                'color': 'aqua',
                'pub_freq': 10.0,
                'brightness': 0.5
            }],
            remappings = [
                ('led_state', 'j7s_led_state')
            ]
        ),
        launch_ros.actions.Node(
            package='j7s',
            node_executable='j7s-pub',
            node_name='red_pub',
            output='log',
            parameters=[{
                'freq': 1.0,
                'led_index': 1,
                'color': 'red',
                'pub_freq': 10.0,
                'brightness': 0.5
            }],
            remappings = [
                ('led_state', 'j7s_led_state')
            ]
        ),
        launch_ros.actions.Node(
            package='j7s',
            node_executable='j7s-pub',
            node_name='white_pub',
            output='log',
            parameters=[{
                'freq': 2.0,
                'led_index': 2,
                'color': 'white',
                'pub_freq': 10.0,
                'brightness': 0.5
            }],
            remappings = [
                ('led_state', 'j7s_led_state')
            ]
        ),
        launch_ros.actions.Node(
            package='j7s',
            node_executable='j7s-pub',
            node_name='green_pub',
            output='log',
            parameters=[{
                'freq': 2.0,
                'led_index': 3,
                'color': 'green',
                'pub_freq': 10.0,
                'brightness': 0.5
            }],
            remappings = [
                ('led_state', 'j7s_led_state')
            ]
        )
    ])

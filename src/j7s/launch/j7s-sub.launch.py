from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='j7s',
            executable='j7s-sub',
            output='log',
            parameters = [{}],
            remappings = [
                ('led_state', 'j7s_led_state')
            ]
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hp_threshold',
            default_value='200.0',
            description='HP threshold to trigger retreat (below this -> go home)'
        ),
        DeclareLaunchArgument(
            'hp_recovery_ratio',
            default_value='0.90',
            description='HP recovery ratio to resume attack (e.g. 0.90 = 90% of max_hp)'
        ),
        DeclareLaunchArgument(
            'nav_action_name',
            default_value='navigate_to_pose',
            description='Primary NavigateToPose action name'
        ),
        DeclareLaunchArgument(
            'nav_action_fallback',
            default_value='/navigate_to_pose',
            description='Fallback NavigateToPose action name (empty to disable)'
        ),

        Node(
            package='rm_decision',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[{
                'hp_threshold': LaunchConfiguration('hp_threshold'),
                'hp_recovery_ratio': LaunchConfiguration('hp_recovery_ratio'),
                'nav_action_name': LaunchConfiguration('nav_action_name'),
                'nav_action_fallback': LaunchConfiguration('nav_action_fallback'),
            }],
            remappings=[
                # 对接 rm_serial_driver 发布的话题
                ('game_status', '/serial/game_status'),
                ('robot_status', '/serial/game_robot_status'),
            ]
        )
    ])

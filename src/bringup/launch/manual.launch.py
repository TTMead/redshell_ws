
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # ==== Motor IO ====
        launch_ros.actions.Node(
            package='redshell_io',
            executable='redshell_interface',
        ),

        # ==== Joystick ====
        launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',

            # Right-Handed 1 stick configuration
            # parameters=[{"enable_button": 5}, {"axis_linear.x": 4}, {"axis_angular.yaw": 3}],

            # 2 stick configuration
            parameters=[{"use_sim_time": True}, {"enable_button": 5}, {"axis_linear.x": 4}, {"axis_angular.yaw": 0}],
        ),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            parameters=[{"use_sim_time": True}, {"autorepeat_rate": 20.0}],
        ),

        # ==== State Estimation ====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('state_estimation'), 'launch/state_estimation.launch.py')
            )
        )
    ])

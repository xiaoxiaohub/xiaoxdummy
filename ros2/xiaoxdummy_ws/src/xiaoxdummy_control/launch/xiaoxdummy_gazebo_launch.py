from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    ld = LaunchDescription()

    # 机器人URDF描述
    ld.add_action(
        DeclareLaunchArgument(
            'robot_desc',
            default_value='',
            description='Robot description in URDF format',
        )
    )

    robot_desc = LaunchConfiguration('robot_desc')  # 会被替换为实际字符串


    # Gazebo simulation
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch'),
                 '/gz_sim.launch.py']
            ),
            launch_arguments=[
                (
                    'gz_args',
                    'empty.sdf -r'
                )
            ]
        )
    )


    # 在 Gazebo 中生成机器人
    ld.add_action(
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', robot_desc,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0',
                '-name', 'xiaoxdummy'
            ]
        )
    )


    # Bridge：同步 Gazebo 与 ROS2 的时间
    ld.add_action(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
            output='screen'
        )
    )

    return ld

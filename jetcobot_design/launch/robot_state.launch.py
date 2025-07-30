#!/usr/bin/env python3

# from launch import LaunchDescription
# from launch_ros.actions import Node
# import os
# from launch.substitutions import PathJoinSubstitution, Command
# from launch_ros.substitutions import FindPackageShare



# def generate_launch_description():
#     urdf_file = PathJoinSubstitution([
#         FindPackageShare('jetcobot_design'),
#         'urdf',
#         'jetcobot.urdf'
#     ])

#     # URDF 파일을 읽어 문자열로 로드
#     import os
#     urdf_path = os.path.join(
#         os.path.dirname(__file__),
#         '..', 'urdf', 'jetcobot.urdf'
#     )
#     with open(urdf_path, 'r') as infp:
#         robot_desc = infp.read()

#     return LaunchDescription([
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='screen',
#             parameters=[{'robot_description': robot_desc}]
#         ),
#         Node(
#             package='joint_state_publisher',
#             executable='joint_state_publisher',
#             name='joint_state_publisher',
#             output='screen'
#         ),
#     ])




#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = os.path.join(
        FindPackageShare('jetcobot_design').find('jetcobot_design'),
        'urdf',
        'jetcobot.urdf'
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
    ])

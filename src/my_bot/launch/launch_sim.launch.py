import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'my_bot'

    # Declare world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='maze_world_scaled.world',
        description='World file name'
    )

    world = LaunchConfiguration('world')

    # Robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': PathJoinSubstitution([
                get_package_share_directory(package_name),
                'worlds',
                world
            ])
        }.items()
    )

    # Spawn robot at maze start position (top-left corner: -5, 5)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot',
            '-x', '-5.0',
            '-y',  '5.0',
            '-z',  '0.1',
            '-Y',  '0'
        ],
        output='screen'
    )

    # Maze solver node
    maze_solver = Node(
        package='my_bot',
        executable='maze_solver.py',
        name='maze_solver',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        rsp,
        gazebo,
        spawn_entity,
        maze_solver,
    ])

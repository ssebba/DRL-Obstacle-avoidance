#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('oa_drl_control'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    lidar_rate = LaunchConfiguration('lidar_rate', default='33')

    world_file = LaunchConfiguration('world_file')
    gui = LaunchConfiguration('gui')

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value='test_env.world',
        description='World to test in: test_env.world or unseen_env.world'
    )
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start the Gazebo client'
    )

    world = PathJoinSubstitution([
        get_package_share_directory('oa_drl_control'), 'worlds', world_file
    ])

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui)
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'lidar_rate': lidar_rate
        }.items()
    )

    filter_data_cmd = Node(
        package='oa_drl_control',
        executable='filter_lidar',
        name='filter_data_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # The controller calls /randomize_robot_pose after every collision. The respawner
    # must parse the same world, and starts once Gazebo has loaded it and the robot.
    respawner_cmd = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='oa_drl_control',
                executable='respawner',
                name='respawner_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'world_file': world_file}]
            )
        ]
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    ld.add_action(filter_data_cmd)
    ld.add_action(respawner_cmd)

    return ld

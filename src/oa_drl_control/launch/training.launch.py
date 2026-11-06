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
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('oa_drl_control'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-5.0')
    y_pose = LaunchConfiguration('y_pose', default='3.5')
    lidar_rate = LaunchConfiguration('lidar_rate')

    declare_lidar_rate_cmd = DeclareLaunchArgument(
        'lidar_rate',
        default_value='33',
        description='LiDAR update rate [Hz]'
    )

    world = os.path.join(
        get_package_share_directory('oa_drl_control'),
        'worlds',
        'training_env.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Empty temporary SDF file: its path is referenced by both the spawn Node and the OpaqueFunction
    _temp_sdf = tempfile.NamedTemporaryFile(delete=False, suffix='.sdf')
    _temp_sdf_path = _temp_sdf.name
    _temp_sdf.close()

    # Spawn node defined at top level so it can be used as target_action of OnProcessExit.
    # It reads the temporary file, patched by the OpaqueFunction before execution.
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_turtlebot3',
        arguments=[
            '-entity', 'burger',
            '-file', _temp_sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
        ],
        output='screen',
    )

    # Patch the SDF with the requested lidar_rate and write it to the temporary file before spawning
    def patch_sdf_with_lidar_rate(context, *args, **kwargs):
        TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
        model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
        urdf_path = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models',
            model_folder,
            'model.sdf'
        )
        lidar_rate_val = lidar_rate.perform(context)
        with open(urdf_path, 'r') as f:
            xml_content = f.read()
        xml_content = xml_content.replace(
            '<update_rate>10</update_rate>',
            f'<update_rate>{lidar_rate_val}</update_rate>'
        )
        with open(_temp_sdf_path, 'wb') as f:
            f.write(xml_content.encode('utf-8'))
        return []

    # The OpaqueFunction performs the patching; spawn_node is started separately afterwards
    patch_sdf_cmd = OpaqueFunction(function=patch_sdf_with_lidar_rate)

    # filter_data_cmd starts once the robot spawn has completed
    filter_data_cmd = Node(
        package='oa_drl_control',
        executable='filter_lidar',
        name='filter_data_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # respawner_cmd starts after filter_data_cmd, with enough delay for Gazebo
    # to have loaded the world and the robot model
    respawner_cmd = Node(
        package='oa_drl_control',
        executable='respawner',
        name='respawner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Start filter_data_cmd right after spawn_node exits
    start_filter_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_node,
            on_exit=[filter_data_cmd]
        )
    )

    # Start the respawner after the spawn, delayed to let the filter initialise
    start_respawner_after_filter = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_node,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[respawner_cmd]
                )
            ]
        )
    )

    ld = LaunchDescription()

    # 0. Declare launch arguments
    ld.add_action(declare_lidar_rate_cmd)

    # 1. Start Gazebo (headless) and robot_state_publisher in parallel
    ld.add_action(gzserver_cmd)
    #ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)

    # 2a. Patch the SDF with lidar_rate (must happen before the spawn)
    ld.add_action(patch_sdf_cmd)

    # 2b. Spawn the robot (uses the already patched SDF)
    ld.add_action(spawn_node)

    # 3. Dopo lo spawn: avvia filter_data_cmd
    ld.add_action(start_filter_after_spawn)

    # 4. After spawn + 2 s: start the respawner
    ld.add_action(start_respawner_after_filter)

    return ld

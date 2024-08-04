# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration


from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    is_sim = False


    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_project_description = get_package_share_directory('ros_gz_example_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the SDF file from "description" package
    # sdf_file  =  os.path.join(pkg_project_description, 'models', 'diff_drive_urdf', 'model.urdf.xacro')
    # with open(sdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    xacro_file = os.path.join(pkg_project_description, 'models', 'diff_drive_urdf', 'model.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file])

    if (is_sim == True):
        use_sim_time = True

        # Setup to launch the simulator and Gazebo world
        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': PathJoinSubstitution([
                pkg_project_gazebo,
                'worlds',
                'diff_drive.sdf'
            ])}.items(),
        )
    
        # Bridge ROS topics and Gazebo messages for establishing communication
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_example_bridge.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }],
            remappings=[
                ('/diff_drive/cmd_vel', '/cmd_vel_nav'),
            ],
            output='screen'
        )

        ignition_spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-string', robot_description_config,
                    '-name', 'diff_drive',
                    '-allow_renaming', 'true',
                    '-x', '-1',
                    '-y', '0',
                    '-z', '0.01'],

            )
    else:
        use_sim_time = False

        robot_controllers = os.path.join(pkg_project_bringup, 'config', 'diff_drive_controller_velocity.yaml')
        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_controllers],
            output="both",
            remappings=[
                ("~/robot_description", "/robot_description"),
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )

        rplidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')),
            launch_arguments={'serial_port': '/dev/ttyUSB0',
                                'serial_baudrate': '460800',
                                'frame_id': 'lidar_link'
                          }.items(),
        )


    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description_config}],
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': os.path.join(pkg_project_bringup, 'config', 'mapper_params_online_async.yaml')}.items(),
    )
    
    joy = Node(
        package='joy',
        executable='joy_node',
        parameters=[os.path.join(pkg_project_bringup, 'config', 'joy-params.yaml')],
        output='screen'
    )

    joy_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch', 'teleop-launch.py')),
        launch_arguments={'config_filepath': os.path.join(pkg_project_bringup, 'config', 'ps3.config.yaml'),
                            "joy_vel": '/diff_drive_base_controller/cmd_vel_unstamped'}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={'params_file': os.path.join(pkg_project_bringup, 'config', 'nav2_params.yaml'),
                        "map":os.path.join(pkg_project_bringup, 'config', 'my_map2.yaml'),
                        'use_sim_time': str(use_sim_time),
                        'slam': 'True',
                        }.items(),
    )

    # joy = Node(
    #     package='teleop_twist_joy',
    #     executable='teleop_twist_joy_node',
    #     parameters=[os.path.join(pkg_project_bringup, 'config', 'ps3.config.yaml')],
    #     output='screen'
    # )

    my_node = Node(
        package='my_package',
        executable='my_node',
        output='both'
    )




    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )


    if (is_sim == True):
        return LaunchDescription([
            gz_sim,
            DeclareLaunchArgument('rviz', default_value='true',
                                description='Open RViz.'),
            bridge,
            robot_state_publisher,

            #joy,

            ignition_spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner,

    #        joy_teleop,
    #        slam
            nav2,
            my_node,

            rviz
        ])
    else:
        return LaunchDescription([
            DeclareLaunchArgument('rviz', default_value='true',
                                description='Open RViz.'),
            robot_state_publisher,


            rplidar,

            #joy,
            control_node,
            diff_drive_spawner,
            joint_broad_spawner,

    #        joy_teleop,
    #        slam
            nav2,
            my_node,

      #      rviz
        ])

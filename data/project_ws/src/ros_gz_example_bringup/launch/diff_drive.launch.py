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
from launch.actions import DeclareLaunchArgument, TimerAction
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
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    if 'IS_DEBUGGER' in os.environ:
        is_debugger = True
    else:
        is_debugger = False

    if 'IS_SIM' in os.environ:
        is_sim = True
    else:
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
    robot_description_config = Command(['xacro ', xacro_file, ' sim_mode:=', str(is_sim)])

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
                    '-z', '0.15'],

            )

    else:
        use_sim_time = False

        # Visualize in RViz
        usb_camera = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='both',
            arguments=['--ros-args', '--params-file', os.path.join(pkg_project_bringup, 'config', 'camera.yaml')],
        )
        # usb_camera = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('usb_cam'), 'launch', 'camera.launch.py')),
        #     launch_arguments={'params_file': os.path.join(pkg_project_bringup, 'config', 'camera.yaml')
        #                 }.items(),
        # )

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

        bno080_node = Node(
            package='bno080',
            executable='bno080_node',
            name='bno080_node',
            output='both',
            )
            
        compress_node = Node(
            package='camera_process2',
            executable='compress_node',
            name='compress_node',
            output='both',
            )

        rplidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')),
            launch_arguments={'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_209c999d04e6ed11a087dca80b2af5ab-if00-port0',
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
       parameters=[{'use_sim_time': use_sim_time}],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )


    if (is_sim == True):
        nav2_config_file = 'nav2_params_sim.yaml'
    else:
        nav2_config_file = 'nav2_params_rpi.yaml'
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={'params_file': os.path.join(pkg_project_bringup, 'config', nav2_config_file),
                        "map":os.path.join(pkg_project_bringup, 'config', 'my_map2.yaml'),
                        'use_sim_time': str(use_sim_time),
                        'slam': 'True',
                        'use_composition': 'False',
                        }.items(),
    )
    # joy = Node(
    #     package='joy',
    #     executable='joy_node',
    #     parameters=[os.path.join(pkg_project_bringup, 'config', 'joy-params.yaml')],
    #     output='screen'
    # )

    # joy_teleop = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch', 'teleop-launch.py')),
    #     launch_arguments={'config_filepath': os.path.join(pkg_project_bringup, 'config', 'ps3.config.yaml'),
    #                         "joy_vel": '/diff_drive_base_controller/cmd_vel_unstamped'}.items(),
    # )



    # joy = Node(
    #     package='teleop_twist_joy',
    #     executable='teleop_twist_joy_node',
    #     parameters=[os.path.join(pkg_project_bringup, 'config', 'ps3.config.yaml')],
    #     output='screen'
    # )

    my_node = Node(
        package='my_package',
        executable='my_node',
        output='both',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    map_process_node = Node(
        package='map_process',
        executable='my_node',
        output='both',
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # imu_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["imu_sensor_broadcaster"],
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )


    madgwick_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='both',
        parameters=[os.path.join(pkg_project_bringup, 'config', 'imu_filter.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('/imu/data_raw', '/bno080/imu'),
                    ('/imu/mag', '/bno080/mag')]
    )

    robot_localization_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='both',
        parameters=[os.path.join(pkg_project_bringup, 'config', 'ekf.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/local')]    
    )

    robot_localization_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='both',
        parameters=[os.path.join(pkg_project_bringup, 'config', 'ekf.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/global')]  
    )

    # slam = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
    #     launch_arguments={
    #         'slam_params_file': os.path.join(pkg_project_bringup, 'config', 'mapper_params_online_async.yaml'),
    #         'use_sim_time': str(use_sim_time)        
    #     }.items(),
    # )

    bno080_calibration_node = Node(
        package='bno080',
        executable='calibration_node',
        name='calibration_node',
        output='both',
        )

    foxglove_bridge = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                                            get_package_share_directory('foxglove_bridge'),
                                            'launch',
                                            'foxglove_bridge_launch.xml'
                                        ))
        )


    camera_udp_publisher_node = Node(
        package='camera_udp_publisher',
        executable='my_node',
        name='camera_udp_publisher',
        output='both',
        )



    if (is_debugger == False):
        if (is_sim == True):
            # Delay of 3 seconds after gz_sim
            delayed_actions = TimerAction(
                period=30.0,
                actions=[
                    diff_drive_spawner,
                    joint_broad_spawner,
                    madgwick_filter,
                    robot_localization_odom,
                    robot_localization_map,
                    nav2,

                    # custom node
                    my_node,
                    map_process_node,
                    camera_udp_publisher_node,

                    # debuging tools
                    foxglove_bridge,
                    rviz
                ]
            )

            return LaunchDescription([
                DeclareLaunchArgument('rviz', default_value='true',
                                    description='Open RViz.'),
                gz_sim,
                bridge,
                ignition_spawn_entity,
                robot_state_publisher,
                
                delayed_actions
            ])
        else:
            return LaunchDescription([
                DeclareLaunchArgument('rviz', default_value='true',
                                    description='Open RViz.'),
                robot_state_publisher,

                # hw depend
                rplidar,
                usb_camera,
                bno080_node,
                compress_node,

                control_node,
                diff_drive_spawner,
                joint_broad_spawner,
                madgwick_filter,
                robot_localization_odom,
                robot_localization_map,
                nav2,

                # custom node
                my_node,
                map_process_node,

                # debuging tools
                foxglove_bridge
            ])
    else:
        return LaunchDescription([
                DeclareLaunchArgument('rviz', default_value='true',
                                    description='Open RViz.'),
                # debuging tools
                rviz
            ])
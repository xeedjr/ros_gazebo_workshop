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
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node, SetParameter, SetRemap

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

    is_sim = True

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

        # image_bridge = Node(
        #     package='ros_gz_image',
        #     executable='image_bridge',
        #     arguments=[
        #         '/camera_left',
        #         '/camera_right'
        #     ],
        #     output='screen'
        # )

        ignition_spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-string', robot_description_config,
                    '-name', 'diff_drive',
                    '-allow_renaming', 'true',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.0'],

            )

        sim_stereo_camera_node = Node(
            package='stereo_camera_package',
            executable='sim_stereo_camera_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )

        # --- Stereo camera calibration pipeline (replaces Nav2 / RTAB-Map) ---
        # Consumes the stereo images republished by sim_stereo_camera_node from
        # the Gazebo cameras, detects the chessboard, and writes the recovered
        # intrinsics to new yaml files under calibration_sim/.
        stereo_calibration_node = Node(
            package='stereo_camera_package',
            executable='stereo_calibration_node',
            name='stereo_calibration_node',
            output='screen',
            parameters=[{
                'pattern_cols': 9,
                'pattern_rows': 6,
                'square': 0.04,
                'min_poses': 20,
                'max_capture_time': 180.0,
                'output_dir': '/root/data/project_ws/calibration_sim',
            }],
        )

        # Teleports the kinematic calibration_board through many poses so the
        # calibration node sees the chessboard from diverse viewpoints.
        chessboard_mover_node = Node(
            package='stereo_camera_package',
            executable='chessboard_mover',
            name='chessboard_mover',
            output='screen',
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
                        # No static map: the global costmap consumes the occupancy grid
                        # that RTAB-Map publishes on /map.
                        "map": '',
                        'use_sim_time': str(use_sim_time),
                        'slam': 'False',
                        # Disable Nav2's AMCL (which would subscribe to the simulated
                        # /scan topic) and the static map_server. RTAB-Map provides
                        # localization via its map -> odom -> base_link TF tree and /odom.
                        'use_localization': 'False',
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
                                        )),
                                        launch_arguments={
                                            'port': '8769'  # replace 8766 with your desired port
                                        }.items()
        )


    # # Launch image_proc node for image processing
    # image_proc_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('image_proc'), 'launch', 'image_proc.launch.py')
    #     )
    # )

    stereo_camera_node = Node(
        package='stereo_camera_package',
        executable='stereo_camera_node',
        output='both',
        parameters=[{'use_sim_time': use_sim_time}],
    )


    image_proc_rect_left_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_node_rect',
        namespace='stereo/stereo_camera/left',
        output='screen',
        remappings=[
            ('image', '/stereo/stereo_camera/left/image_raw'),
            ('camera_info', '/stereo/stereo_camera/left/camera_info'),
            ('image_rect', '/stereo/stereo_camera/left/image_rect')
        ]
    )

    image_proc_rect_right_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_node_rect',
        namespace='stereo/stereo_camera/right',
        output='screen',
        remappings=[
            ('image', '/stereo/stereo_camera/right/image_raw'),
            ('camera_info', '/stereo/stereo_camera/right/camera_info'),
            ('image_rect', '/stereo/stereo_camera/right/image_rect')
        ]
    )

    stereo_image_proc2 = Node(   
        package='stereo_image_proc',
        executable='disparity_node',
        name='disparity_node',
        namespace='stereo',
        remappings=[
            ('left/image_rect', '/stereo/stereo_camera/left/image_rect'),
            ('left/camera_info', '/stereo/stereo_camera/left/camera_info'),
            ('right/image_rect', '/stereo/stereo_camera/right/image_rect'),
            ('right/camera_info', '/stereo/stereo_camera/right/camera_info')
        ],
        parameters=[
            {'queue_size': 5},
            {'approximate_sync': False},
            {'use_system_default_qos': True}
        ]
    )


    ## ros2 run rtabmap_odom stereo_odometry --params

    parameters_odom=[{
          'frame_id':'base_link',
          'subscribe_stereo':True,
          'subscribe_odom_info':True,
          'wait_imu_to_init':False,
          'use_sim_time':use_sim_time,
          'always_check_imu_tf': False,
          'Rtabmap/ImagesAlreadyRectified':"true", 
          #'Vis/FeatureType': "8",
          'publish_tf':True}]

    parameters_slam=[{
          'frame_id':'base_link',
          # Publish the map -> odom transform so the TF tree is map -> odom -> base_link.
          # This is the frame Nav2's global costmap (global_frame: map) needs.
          'odom_frame_id':'odom',
          'subscribe_stereo':True,
          'subscribe_odom_info':True,
          'wait_imu_to_init':False,
          'use_sim_time':use_sim_time,
          'always_check_imu_tf': False,
          'Grid/MaxObstacleHeight': "0.35",
          'publish_tf':True}]

    remappings=[
          ('left/image_rect', '/stereo/stereo_camera/left/image_rect'),
          ('left/camera_info', '/stereo/stereo_camera/left/camera_info'),
          ('right/image_rect', '/stereo/stereo_camera/right/image_rect'),
          ('right/camera_info', '/stereo/stereo_camera/right/camera_info')]

    # Visual odometry
    rtabmap_odom_node =   Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=parameters_odom,
            remappings=remappings)

    rtabmap_slam_node = Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters_slam,
            remappings=remappings,
            arguments=['-d'])

    # rtabmap_viz only displays the map; it must not publish the map -> odom TF
    # (rtabmap_slam already does), otherwise two nodes would publish the same frame.
    rtabmap_viz_node = Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters_slam + [{'publish_tf': False}],
            remappings=remappings)




    if (is_debugger == False):
        if (is_sim == True):
            # Start the RTAB-Map pipeline (and controllers) first so the map frame
            # (map -> odom -> base_link) is established before Nav2 activates.
            delayed_actions = TimerAction(
                period=30.0,
                actions=[
                    diff_drive_spawner,
                    joint_broad_spawner,

                    # custom node
                    my_node,

                    # Stereo camera calibration pipeline (no Nav2 / RTAB-Map).
                    # The calibration node consumes the stereo images and the
                    # mover teleports the chessboard for diverse poses.
                    stereo_calibration_node,
                    chessboard_mover_node,
                ]
            )

            # Nav2 starts later so its costmaps can find the map frame in TF when
            # the lifecycle manager activates them.
            delayed_nav2_actions = TimerAction(
                period=60.0,
                actions=[nav2]
            )

            # Nav2 lifecycle transitions (configure + activate) are driven
            # explicitly, because the nav2 lifecycle_manager (autostart) stalls
            # forever waiting for collision_monitor, which is not launched here
            # (its node_names list is hardcoded in nav2's navigation_launch.py).
            # The script waits for the nodes, then configures/activates them with
            # behavior_server active before bt_navigator (so the Spin/BackUp/Wait
            # action servers exist when bt_navigator loads its behavior tree).
            nav2_lifecycle_script = os.path.join(
                pkg_project_bringup, 'launch', 'activate_nav2_lifecycle.sh')
            delayed_nav2_lifecycle = TimerAction(
                period=70.0,
                actions=[
                    ExecuteProcess(
                        cmd=['bash', nav2_lifecycle_script],
                        output='screen',
                    )
                ]
            )

            # Automatically resume (play) the simulator on startup so it does not
            # stay paused waiting for a manual "play" click.
            #
            # The ROS bridge does not expose the sim's world-control service, so we
            # call it directly over gz-transport: /world/<name>/control with a
            # WorldControl(pause=False). The script discovers the world name and
            # retries until the service is up. Toggle with: auto_play:=false
            gz_auto_play_script = '''
import sys, time
for _p in ("/opt/ros/jazzy/opt/gz_transport_vendor/lib/python",
           "/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python"):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import gz.transport as t
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
node = t.Node()
wc = WorldControl()
wc.pause = False
def find_control():
    for s in node.service_list():
        if s.startswith("/world/") and s.endswith("/control"):
            return s
    return None
done = False
for _ in range(30):
    svc = find_control()
    if svc:
        try:
            ok, _ = node.request(svc, wc, WorldControl, Boolean, 2000)
            if ok:
                print("Gazebo auto_play: simulation resumed via " + svc)
                done = True
                break
        except Exception:
            pass
    time.sleep(1)
print("Gazebo auto_play: done" if done else "Gazebo auto_play: could not resume simulation")
'''
            gz_auto_play = TimerAction(
                period=10.0,
                actions=[ExecuteProcess(
                    cmd=['python3', '-c', gz_auto_play_script],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('auto_play')),
                )],
            )

            return LaunchDescription([
                DeclareLaunchArgument('rviz', default_value='true',
                                    description='Open RViz.'),
                DeclareLaunchArgument('auto_play', default_value='true',
                                    description='Automatically resume/play the simulator on startup.'),
                gz_sim,
                bridge,
                # # image_bridge,
                ignition_spawn_entity,
                robot_state_publisher,
                sim_stereo_camera_node,

                gz_auto_play,
                delayed_actions,
                # image_proc_node
            ])
        else:
            delayed_actions = TimerAction(
                period=5.0,
                actions=[
                    image_proc_rect_left_node,
                    image_proc_rect_right_node,
                    stereo_image_proc2,
                ]
            )

            return LaunchDescription([
                DeclareLaunchArgument('rviz', default_value='true',
                                    description='Open RViz.'),
                robot_state_publisher,

                stereo_camera_node,
                
                delayed_actions,

                rtabmap_odom_node,
                rtabmap_slam_node,
                rtabmap_viz_node,

                rviz,
            ])
    else:
        return LaunchDescription([
                DeclareLaunchArgument('rviz', default_value='true',
                                    description='Open RViz.'),
                # debuging tools
                rviz
            ])
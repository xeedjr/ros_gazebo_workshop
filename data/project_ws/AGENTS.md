# AGENTS.md

## System startup — main entrypoint

The system **must** be started using `build2.sh`. It is the single main entrypoint for this workspace:

```bash
bash build2.sh
```

What `build2.sh` does, in order:

1. `apt update`
2. Sources `/opt/ros/jazzy/setup.bash` (ROS 2 Jazzy)
3. `rosdep init` + `rosdep update`
4. `rosdep install --from-paths src --ignore-src -r -i -y --rosdistro jazzy`
5. Re-sources `/opt/ros/jazzy/setup.bash`
6. `colcon build --cmake-args -DBUILD_TESTING=ON`
7. Sources `./install/setup.sh`
8. Launches the system:
   `ros2 launch ros_gz_example_bringup diff_drive.launch.py`

Do not run individual steps (colcon build, ros2 launch, etc.) manually for normal operation — use `build2.sh` so the environment, dependencies, and build are consistent.

### ⚠️ Single instance only — never run `build2.sh` in parallel

`build2.sh` starts a **full single-instance** system (one Gazebo sim, one RTAB-Map, one Nav2, one RViz, one Foxglove bridge bound to port 8769, all on ROS_DOMAIN_ID 0). A second concurrent instance spawns duplicate/conflicting nodes (duplicate topic publishers, a second Foxglove bridge that fails to bind 8769, extra RViz windows, and both writing the same `rtabmap.db`).

- **Never launch `build2.sh` more than once at a time.** Before starting, confirm none is already running: `ps aux | grep -E "build2.sh|ros2 launch|gz sim"`.
- **Do not "watch" a running instance by re-running its command.** A background-process `monitor` (or any tool that *runs* a command) given `bash build2.sh` **starts a brand-new instance** — it does not attach to the one already running. To observe an already-started instance, use the background-process `logs`/`status` action on the **existing process ID** returned by `start`.
- If a second instance was started by mistake, **stop all of them** (background-process `stop`, which kills the whole process tree), verify nothing is left running, then start exactly one clean instance.

## Project overview

ROS 2 (Jazzy) + Gazebo simulation workspace for a differential-drive robot with:

- **Simulation**: Gazebo (gz_sim) with the `diff_drive` robot spawned from `ros_gz_example_description` xacro
- **Navigation**: Nav2 (no static map; global costmap consumes the occupancy grid RTAB-Map publishes on `/map`)
- **SLAM / localization**: RTAB-Map (stereo visual odometry + SLAM), providing the `map -> odom -> base_link` TF tree and `/odom`
- **Sensors (sim)**: stereo camera, IMU (Madgwick filter)
- **Bridges**: `ros_gz_bridge` parameter bridge (`/diff_drive/cmd_vel` remapped to `/cmd_vel_nav`), Foxglove bridge on port 8769

## Workspace layout

```
/root/data/project_ws/
├── build2.sh                  # main entrypoint (see above)
├── src/
│   ├── ros_gz_example_bringup # launch files + config (the launched package)
│   ├── ros_gz_example_gazebo  # Gazebo world (diff_drive.sdf)
│   ├── ros_gz_example_description # robot xacro/URDF
│   ├── ros_gz_example_application
│   ├── bno080                   # BNO080 IMU driver + calibration node
│   ├── stereo_camera_package    # stereo camera nodes (sim + real)
│   ├── camera_process2
│   ├── camera_udp_publisher
│   ├── capture
│   ├── hardware_link2
│   ├── map_process
│   ├── my_package
│   └── ...
├── calibration.txt
├── left.yaml / right.yaml      # stereo camera calibration
├── warehouse.*                 # warehouse world assets (map, posegraph, ...)
```

## Launch

- Main launch file: `src/ros_gz_example_bringup/launch/diff_drive.launch.py`
- `is_sim` is hard-coded to `True` in the launch file (simulation mode); the real-robot branch exists but is not the active path.
- Launch arguments:
  - `rviz` (default `true`) — open RViz
  - `auto_play` (default `true`) — auto-resume the Gazebo simulation via a `gz-transport` WorldControl call

### Startup ordering (sim branch)

- `t=30s`: diff drive spawner, joint state broadcaster, Madgwick IMU filter, RTAB-Map pipeline (rectify → stereo disparity → stereo_odometry → rtabmap SLAM → rtabmap_viz), RViz
- `t=60s`: Nav2 bringup (delayed so the `map` frame exists in TF before Nav2 activates)
- `t=10s`: Gazebo auto-play script

## Key configs

- `src/ros_gz_example_bringup/config/`
  - `ros_gz_example_bridge.yaml` — ROS↔Gazebo bridge config
  - `nav2_params_sim.yaml` / `nav2_params_rpi.yaml` — Nav2 params (sim vs RPi)
  - `ekf.yaml` — robot_localization (note: EKF nodes are commented out in the launch; RTAB-Map provides odom)
  - `imu_filter.yaml` — Madgwick filter
  - `diff_drive_controller_velocity.yaml` — controller manager
  - `diff_drive.rviz` — RViz config

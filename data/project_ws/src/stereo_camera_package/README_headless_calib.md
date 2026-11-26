# Headless Stereo Camera Calibration Node

Headless ROS 2 node that replaces the GUI `stereo_calibrator` from the
`camera_calibration` package. The same `StereoCalibrator` engine is used;
GUI buttons become ROS services and the image panels become ROS topics.

## Launch

```bash
ros2 run stereo_camera_package stereo_calib_headless_node
```

Optional parameters (pass with `--ros-args -p`):

| Parameter              | Type  | Default                              | Description                          |
|------------------------|-------|--------------------------------------|--------------------------------------|
| `camera_name`          | str   | `"stereo_camera"`                    | Name used in calibration output      |
| `pattern_cols`         | int   | `9`                                  | Chessboard columns (interior)        |
| `pattern_rows`         | int   | `6`                                  | Chessboard rows (interior)           |
| `square`               | float | `0.04`                               | Square size in metres                |
| `left_topic`   | str    | `/stereo/stereo_camera/left/image_raw_raw`  | Left image topic (bridge raw, unprocessed) |
| `right_topic`  | str    | `/stereo/stereo_camera/right/image_raw_raw` | Right image topic (bridge raw, unprocessed) |
| `checkerboard_flags`   | int   | `cv2.CALIB_CB_FAST_CHECK`            | OpenCV chessboard flags              |
| `max_chessboard_speed` | float | `-1.0`                               | Max board speed px/s (-1 = disabled) |
| `publish_annotated`    | bool  | `true`                               | Publish annotated images             |
| `annotated_pub_rate`   | float | `5.0`                                | Max Hz for annotated image pubs      |

Example:

```bash
ros2 run stereo_camera_package stereo_calib_headless_node \
  --ros-args -p pattern_cols:=11 -p pattern_rows:=8 -p square:=0.03
```

## ROS Interfaces

### Services (all `std_srvs/srv/Trigger`)

| Service                    | Action                                    |
|----------------------------|-------------------------------------------|
| `/stereo_calib/calibrate`  | Run stereo calibration (requires READY)   |
| `/stereo_calib/save`       | Save to `/tmp/calibrationdata.tar.gz`     |
| `/stereo_calib/clear`      | Clear all accumulated samples             |
| `/stereo_calib/cancel`     | Shut the node down                        |

### Topics

| Topic                          | Type                | Description                        |
|--------------------------------|---------------------|------------------------------------|
| `/stereo_calib/left_annotated` | `sensor_msgs/Image` | Left view with detected corners    |
| `/stereo_calib/right_annotated`| `sensor_msgs/Image` | Right view with detected corners   |
| `/stereo_calib/status`         | `std_msgs/String`   | `[STATE] good=N/40`                |
| `/stereo_calib/result`         | `std_msgs/String`   | Calibration report (K, D, R, T...) |

## Workflow

1. **Start the node** and make sure the chessboard is visible to both cameras.

2. **Monitor progress:**
   ```bash
   ros2 topic echo /stereo_calib/status
   ```
   Wait until the state changes from `[COLLECTING]` to `[READY]`.
   The node needs either 40 good samples or full parameter coverage
   (position, size, skew diversity).

3. **View annotated images** (in Foxglove, RViz, or `rqt_image_view`):
   ```bash
   ros2 run rqt_image_view rqt_image_view /stereo_calib/left_annotated
   ```
   Green corners = sample added to database.

4. **Calibrate:**
   ```bash
   ros2 service call /stereo_calib/calibrate std_srvs/srv/Trigger
   ```
   The response contains the full calibration report. The report is also
   published on `/stereo_calib/result`.

5. **Save:**
   ```bash
   ros2 service call /stereo_calib/save std_srvs/srv/Trigger
   ```
   Writes `/tmp/calibrationdata.tar.gz` (standard ROS camera calibration
   format, usable with `ros2 run camera_calibration cameracalibrator`
   tools or extracted manually).

6. **Reset** (if you want to start over without restarting the node):
   ```bash
   ros2 service call /stereo_calib/clear std_srvs/srv/Trigger
   ```

## Collecting Annotated Images to Disk

To save every annotated frame to a folder while calibrating:

```bash
python3 collect_calib_images.py /path/to/output_dir
```

Creates `output_dir/left/00000.png`, `output_dir/right/00000.png`, etc.

## Using with the Gazebo Sim (this workspace)

The sim exposes stereo images on the ros_gz_bridge raw (unprocessed) topics,
which this node consumes directly:
- `/stereo/stereo_camera/left/image_raw_raw`
- `/stereo/stereo_camera/right/image_raw_raw`

The `chessboard_mover` node teleports a chessboard through 80 poses
(~4 s each, 320 s total). Run both together:

```bash
# Terminal 1: move the board
ros2 run stereo_camera_package chessboard_mover

# Terminal 2: calibrate
ros2 run stereo_camera_package stereo_calib_headless_node

# Terminal 3 (optional): save images
python3 collect_calib_images.py /root/data/project_ws/calib_images
```

The mover may need to be restarted multiple times to collect enough
diverse samples (the `StereoCalibrator` requires more diverse poses than
the older `stereo_calibration_node`).

## Notes

- The node patches `camera_calibration.calibrator._get_corners` to use
  `cv2.findChessboardCornersSB` (Shi-Tomasi) instead of the classic
  `cv2.findChessboardCorners`. This is necessary because the classic
  detector fails on Gazebo-simulated images.
- After `do_calibration()` the node recomputes the stereo rectification with
  fresh `cv2.stereoRectify` output buffers (`recompute_stereo_rectify`). In
  OpenCV 4.6 (ROS Jazzy), passing the pre-allocated `self.l.P`/`self.r.P`
  buffers that `StereoCalibrator.set_alpha` uses corrupts the focal-length
  diagonal of the projection matrix (observed `fx` ~1e5 instead of ~253);
  recomputing with fresh buffers is reliable.
- The 80-pose `chessboard_mover` plateaus at ~36 good samples (needs >= 40 to
  reach READY). `chessboard_mover_grid.py` sweeps a wider depth/lateral/height/
  tilt grid and reliably reaches >= 40 in one pass.
- `do_save()` writes to `/tmp/calibrationdata.tar.gz` inside the
  container. Copy it out with:
  ```bash
  docker cp ros_gazebo_workshop-ros2_gazebo-1:/tmp/calibrationdata.tar.gz .
  ```

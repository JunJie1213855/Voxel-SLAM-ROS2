# Voxel-SLAM (ROS 2)

## 1. Introduction

**Voxel-SLAM** is a complete, accurate, and versatile LiDAR-inertial SLAM system that fully utilizes short-term, mid-term, long-term, and multi-map data associations. It includes five modules: initialization, odometry, local mapping, loop closure, and global mapping. The initialization can provide accurate states and local map in a static or dynamic initial state. The odometry estimates current states and detect potential system divergence. The local mapping refine the states and local map within the sliding window by a LiDAR-inertial BA. The loop closure can detect in multiple sessions. The global mapping refine the global map with an efficient hierarchical global BA. The system overview is:

<div align="center">
    <a href="https://youtu.be/Cg9W01aIUzE" target="_blank">
    <img src="./figure/systemoverview.png" width = 60% >
    </a>
</div>

The video of **Voxel-SLAM** is available on [YouTube](https://youtu.be/Cg9W01aIUzE).

Related paper is available on [**arXiv**](https://arxiv.org/abs/2410.08935).

Voxel-SLAM has been served as a subsystem to participate in [ICRA HILTI 2023 SLAM Challenge](https://hilti-challenge.com/leader-board-2023.html) (**2nd** place on the LiDAR single-session) and [ICCV 2023 SLAM Challenge](https://superodometry.com/iccv23_challenge_LiI) (**1st** place on the LiDAR inertial track).

---

## 2. About this ROS 2 port

This repository is a **ROS 2 (Humble)** port of the original ROS 1 code base. It is a full ament
workspace and no longer contains any ROS 1 (catkin/roscpp) code, launch files or tooling.

The port is **offline-first**: instead of playing a rosbag alongside a live subscriber, the node
reads a `rosbag2` directory directly (`rosbag2_cpp`), reproduces the original message timing, and
then runs the regular SLAM pipeline.

### 2.1 What the port changes

| Area | Original (ROS 1) | Here (ROS 2) |
|---|---|---|
| Build | `catkin_make` | `colcon build`, `ament_cmake`, package format 3 |
| Middleware | `roscpp`, `ros::NodeHandle` | `rclcpp`, `rclcpp::Node` |
| Parameters | `nh.param` + `rosparam` | declared ROS 2 parameters from YAML / launch |
| Data input | live topics, `rosbag play` | offline `rosbag2_cpp` reader, no `rosbag play` needed |
| RViz plugin | `rviz::Display` (RViz 1) | `rviz_common::Display` (RViz 2) |
| Finalization | `rosparam set finish true` | triggered automatically at end of bag, or `ros2 param set` |

### 2.2 Fixes applied on top of the port

The port itself had several defects; these are fixed here and are worth knowing about:

* **Livox IMU gravity scaling was missing (`ekf_imu.hpp`).** Livox IMU reports linear acceleration
  in **g**, not m/s². The upstream code scales it with `scale_gravity = G_m_s2` when
  `imu_topic == "/livox/imu"`; that line was lost during the port, leaving `scale_gravity = 1.0`.
  Gravity was then estimated ~20× too small and **initialization never converged** — the whole bag
  would play through with zero odometry output. Restored.
* **Livox LiDAR support was missing entirely.** The `LID_TYPE` enum had no `LIVOX` entry, so the
  shipped Livox configs (`lidar_type: 0`) fell into `default:` and exited with `Lidar Type Error`.
  The enum numbering now matches upstream (`LIVOX=0`), a `livox_handler` was added, and the bag
  reader converts `livox_ros_driver2/msg/CustomMsg` into a `PointCloud2` transparently.
* **The bag reader only understood `sensor_msgs/msg/PointCloud2`.** It now looks up the recorded
  topic type and deserializes `CustomMsg` or `PointCloud2` accordingly. Feeding one into the other
  silently produced garbage points rather than an error.
* **Node name mismatch.** The node was created as `cmn_voxel` while launch files and YAML used
  `voxelslam`, so parameters only loaded via a `__node:=` remap. Renamed to `voxelslam`.
* **Missing extrinsics caused a segfault.** `extrinsic_tran` / `extrinsic_rota` defaulted to empty
  vectors and were indexed unguarded. They now default to zero translation / identity rotation and
  are validated with a clear error message.
* **A bad bag path aborted the process.** `rosbag2_cpp::Reader::open()` throws, and on the worker
  thread that called `std::terminate`. It is now caught, reported, and unwinds the run cleanly.
* **The RViz configs referenced plugins that do not exist in Humble.** `rviz_default_plugins/Group`
  (a container holding several point-cloud displays) and `rviz_common/ToolProperties` (Humble spells
  it `Tool Properties`) both failed to load, silently dropping displays. Fixed in `back.rviz` and
  `back_voxel.rviz`.
* **`voxelslam_pointcloud2` was ported to RViz 2**, including the `pluginlib_export_plugin_description_file`
  registration that ROS 2 requires for plugin discovery.

---

## 3. Prerequisites

* Ubuntu 22.04
* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [PCL 1.12](https://pointclouds.org/)
* [Eigen 3.4](https://eigen.tuxfamily.org/index.php?title=Main_Page)
* [GTSAM ≥ 4.2](https://github.com/borglab/gtsam/releases) — the `ros-humble-gtsam` package works
* [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) — **required**, provides the
  `livox_ros_driver2/msg/CustomMsg` type. Build it in its own workspace and source it before building
  this one.

ROS 2 dependencies:

```bash
sudo apt install ros-humble-rviz2 ros-humble-rviz-common ros-humble-rviz-default-plugins \
                 ros-humble-rosbag2-cpp ros-humble-pcl-conversions \
                 ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-eigen \
                 ros-humble-tf2-geometry-msgs ros-humble-tf2-sensor-msgs \
                 ros-humble-pluginlib qtbase5-dev
```

---

## 4. Build

```bash
mkdir -p ~/voxelslam_ws/src
cd ~/voxelslam_ws/src
git clone <this repo>

# livox_ros_driver2 must be discoverable for the CustomMsg headers
source /path/to/livox_ws/install/setup.bash

cd ~/voxelslam_ws
colcon build --symlink-install
source install/setup.bash
```

This workspace contains two ROS 2 packages:

* **`voxel_slam`** — the SLAM system. Executable `voxelslam`, node name `voxelslam`.
* **`voxelslam_pointcloud2`** — the custom RViz2 display plugin (see section 6).

---

## 5. Run

### 5.1 General usage

The node reads a **`rosbag2` directory** (a folder containing `metadata.yaml` and a `.db3` file),
not a ROS 1 `.bag`. Pass it with `bag_path`:

```bash
source /opt/ros/humble/setup.bash
source /path/to/livox_ws/install/setup.bash          # if using a Livox LiDAR
source ~/voxelslam_ws/install/setup.bash

ros2 launch voxel_slam vxlm_mid360.launch.py \
    bag_path:=/absolute/path/to/rosbag2_dir \
    bagname:=my_test_run
```

Available launch files — one per LiDAR, each with `bag_path`, `bagname`, `lid_topic` and `rviz`
arguments:

| LiDAR | Launch file | `lidar_type` |
|---|---|---|
| Livox Avia | `vxlm_avia.launch.py` | 0 |
| Livox Mid360 | `vxlm_mid360.launch.py` | 0 |
| Livox Avia (flying) | `vxlm_avia_fly.launch.py` | 0 |
| Velodyne | `vxlm_velodyne.launch.py` | 1 |
| Ouster | `vxlm_ouster.launch.py` | 2 |
| Hesai | `vxlm_hesai.launch.py` | 3 |
| RoboSense | `vxlm_robosense.launch.py` | 4 |

Arguments are injected as parameter overrides, so **you never need to edit the YAML**:

```bash
ros2 launch voxel_slam vxlm_robosense.launch.py \
    lid_topic:=/sensing/lidar/corrected/front_left/points_cropped \
    bag_path:=/absolute/path/to/rosbag_dir \
    bagname:=my_test_run \
    rviz:=false                     # headless, no RViz
```

At the end of the bag the node **automatically** triggers the `finish` sequence, runs the global
bundle adjustment (GBA), saves the map if `is_save_map` is enabled, and then shuts down the launch
(including RViz). To trigger it manually mid-run:

```bash
ros2 param set /voxelslam finish true
```

> **ROS 1 bags must be converted first.** A `.bag` file cannot be read by this node. Convert it, e.g.
> with [`rosbags-convert`](https://ternaris.gitlab.io/rosbags/topics/rosbags_convert.html)
> (note: the source must be passed with `--src`, there is no positional form):
> ```bash
> pip install rosbags
> rosbags-convert --src my_recording.bag --dst my_recording_ros2/
> ```

### 5.2 Configuration

Sensor parameters live in `Voxel_Slam_ROS2/VoxelSLAM/config/<sensor>.yaml`. The `General` section is
the one you are most likely to touch:

```yaml
General:
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"
  save_path: "/path/to/save/"     # only used when is_save_map: 1
  bagname: "my_test_run"
  lidar_type: 0
  blind: 0.5
  point_filter_num: 3
  extrinsic_tran: [-0.011, -0.02329, 0.04412]              # LiDAR -> IMU translation
  extrinsic_rota: [1,0,0, 0,1,0, 0,0,1]                    # row-major rotation
  is_save_map: 0
```

`extrinsic_tran` must have 3 elements and `extrinsic_rota` 9; the node validates this and exits with
a message if they are wrong.

---

## 6. Datasets

> All downloads below are the **original ROS 1 `.bag` files** from the upstream project. Convert them
> to `rosbag2` (section 5.1) before use, or record your own with the sensor driver.

### 6.1 Livox Avia — online relocalization

Download: [OneDrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/ErEznhkJzTxJiLuJ8AQDGS0BvCy6KsuaWF2D6cnx061GEQ?e=dMRSlf) ([Google Drive](https://drive.google.com/file/d/1LG46i0vreQrMZRap5tJkjKK4IX0t0zfC/view?usp=drive_link))

```bash
ros2 launch voxel_slam vxlm_avia.launch.py bag_path:=/path/to/compus_elevator
```

In the elevator, the system continues to restart until stepping out of the elevator. The blue point
cloud is the map from initialization. Afterwards run the GBA (section 5.1) to refine the global map.

### 6.2 HILTI 2023 — multi-session

Download: [OneDrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/Epp5AQ2Oq1VNhC6MIuCAtN4BJC9jx9VvuVx7VT_cdlvD0A?e=8mXYdc) (quick test); full HILTI 2022/2023 rosbags on the [website](https://hilti-challenge.com/index.html).

Play the sessions in order from `site1_handheld_5` to `site1_handheld_1`; otherwise
`site_handheld_2` and `site_handheld_3` cannot find the loop.

Set the following in `hesai.yaml` before each run (`#` marks a commented-out entry):

```yaml
save_path: "/path/to/save/"
previous_map: "# site1_handheld_5: 0.50,
               # site1_handheld_4: 0.45,
               # site1_handheld_3: 0.30,
               # site1_handheld_2: 0.50"
bagname: "site1_handheld_5"
is_save_map: 1
```

Run each session with the same launch file, pointing `bag_path` at the corresponding bag:

```bash
ros2 launch voxel_slam vxlm_hesai.launch.py bag_path:=/path/to/site1_handheld_5
```

For `site1_handheld_2` onwards, **uncomment the maps you want to load** in `previous_map` — e.g. for
the fourth run the entry should list `site1_handheld_5`, `4` and `3`, leaving `2` commented out. Run
the GBA at the end to get a consistent global map.

### 6.3 MARS dataset

Download: [OneDrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/EpjsGW6coYlMvBWo8TlgJXoBttAuoocLi24V6kw-r_3A8w?e=vVB6RY) (quick test); full MARS rosbags on the [website](https://mars.hku.hk/dataset.html).

```bash
ros2 launch voxel_slam vxlm_avia_fly.launch.py bag_path:=/path/to/HKisland03
```

The beginning of the point cloud is empty and initialization fails until the drone reaches a certain
height — this is expected. For `AMvalley03` this sequence finds loops only with difficulty; run the
GBA to ensure global map consistency.

### 6.4 Livox Mid360

Download: [OneDrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/ErtuXCFhFrBErZxzS5vLASkBJEfgDB9R2CSCgKe8BwhneQ?e=zbr7NL)

```bash
ros2 launch voxel_slam vxlm_mid360.launch.py \
    bag_path:=/path/to/jungle_challenge \
    bagname:=jungle_challenge
```

The bag begins at a violent speed, so initialization may take a while to converge. Live rosbag2
recordings from `livox_ros_driver2` publish `/livox/lidar` as `CustomMsg` rather than `PointCloud2`;
the reader detects the recorded type and converts it (including the per-point offset time used for
de-skewing), so nothing extra is required on your side.

### 6.5 Others

Other types of LiDAR will be released later.

---

## 7. VoxelSLAMPointCloud2 (RViz plugin)

**VoxelSLAMPointCloud2** is a customized RViz2 display. It behaves like the stock "PointCloud2"
display, but **clears the accumulated point cloud map automatically** when it receives an empty point
cloud, regardless of the configured **Decay Time**.

* It is built as part of this workspace — no manual registration needed. The package registers itself
  with `pluginlib` against the `rviz_common` category.
* The launch files already start `rviz2` with `back.rviz`, which uses this plugin.
* To add it manually: RViz2 → "Add" → `voxelslam_pointcloud2/VoxelSLAMPointCloud2`.

---

## 8. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Lidar Type Error` then exit | `lidar_type` in the YAML does not match the sensor. See the table in 5.1. |
| `No storage could be initialized from the inputs` | `bag_path` is not a valid rosbag2 directory (missing `metadata.yaml`), or it points at a ROS 1 `.bag`. Convert it first. |
| Parameters seem ignored | The node must be named `voxelslam` (the top-level YAML key). `ros2 run` without the launch file needs `-r __node:=voxelslam`. |
| `Error: invalid LiDAR-IMU extrinsics` | `extrinsic_tran` needs 3 values, `extrinsic_rota` 9. |
| Initialization never converges (no odometry output, `scale_gravity: 1.000000` in the log) | Livox IMU gravity scaling is not applied — see fix in 2.2. `scale_gravity` should read `9.800000`. |
| RViz reports `failed to load` for a display class | Stale config from a different RViz version; `back.rviz` / `back_voxel.rviz` in this repo are already fixed for Humble. |

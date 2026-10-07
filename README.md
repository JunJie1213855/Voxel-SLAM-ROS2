# Voxel-SLAM: A Complete, Accurate, and Versatile LiDAR-Inertial SLAM System

## 1. Introduction

**Voxel-SLAM** is a complete, accurate, and versatile LiDAR-inertial SLAM system that fully utilizes short-term, mid-term, long-term, and multi-map data associations. It includes five modules: initialization, odometry, local mapping, loop closure, and global mapping. The initialization can provide accurate states and local map in a static or dynamic initial state. The odometry estimates current states and detect potential system divergence. The local mapping refine the states and local map within the sliding window by a LiDAR-inertial BA. The loop closure can detect in multiple sessions. The global mapping refine the global map with an efficient hierarchical global BA. The system overview is:

<div align="center">
    <a href="https://youtu.be/Cg9W01aIUzE" target="_blank">
    <img src="./figure/systemoverview.png" width = 60% >
</div>

### 1.1 Related Video

The video of **Voxel-SLAM** is available on [YouTube](https://youtu.be/Cg9W01aIUzE).

---

### 🚀 Recent Updates (ROS 2 Offline Bag Processing)
The system has been heavily upgraded to support **ROS 2 native offline bag parsing (`rosbag2_cpp`)**. Instead of manually playing bags and risking message loss or synchronization problems, `Voxel-SLAM` now:
1. **Directly Parses `rosbag2` Files**: PointCloud and IMU messages are extracted seamlessly offline. Message speeds remain exactly as originally recorded using multi-threaded synchronization.
2. **Launch Parameters Overrides**: You can now inject arguments (e.g., `lid_topic`, `bag_path`, `bagname`) directly via `ros2 launch` without modifying the global config file format.
3. **Automated Finalization & Map Saving**: Upon finishing the playback of the bag, the system will *automatically* trigger the `finish` sequence, compute global mapping (GBA), save the resulting offline map (if enabled), and cleanly terminate RViz along with the node.

**How to use the dynamic launch arguments:**
```bash
ros2 launch voxel_slam vxlm_robosense.launch.py \
    lid_topic:=/sensing/lidar/corrected/front_left/points_cropped \
    bag_path:=/absolute/path/to/rosbag_dir \
    bagname:=my_test_run
```

---

### 1.2 Related works
The system has been heavily upgraded to support **ROS 2 native offline bag parsing (`rosbag2_cpp`)**. Instead of manually playing bags and risking message loss or synchronization problems, `Voxel-SLAM` now:
1. **Directly Parses `rosbag2` Files**: PointCloud and IMU messages are extracted seamlessly offline. Message speeds remain exactly as originally recorded using multi-threaded synchronization.
2. **Launch Parameters Overrides**: You can now inject arguments (e.g., `lid_topic`, `bag_path`, `bagname`) directly via `ros2 launch` without modifying the global config file format.
3. **Automated Finalization & Map Saving**: Upon finishing the playback of the bag, the system will *automatically* trigger the `finish` sequence, compute global mapping (GBA), save the resulting offline map (if enabled), and cleanly terminate RViz along with the node.

**How to use the dynamic launch arguments:**
```bash
ros2 launch voxel_slam vxlm_robosense.launch.py \
    lid_topic:=/sensing/lidar/corrected/front_left/points_cropped \
    bag_path:=/absolute/path/to/rosbag_dir \
    bagname:=my_test_run
```

---

### 1.2 Related works

Related paper is available on [**arxiv**](https://arxiv.org/abs/2410.08935).

### 1.3 Competitions

Voxel-SLAM has been served as a subsystem to participate in [ICRA HILTI 2023 SLAM Challenge](https://hilti-challenge.com/leader-board-2023.html) (**2nd** place on the LiDAR single-session) and [ICCV 2023 SLAM Challenge](https://superodometry.com/iccv23_challenge_LiI) (**1st** place on the LiDAR inertial track).

## 2. Prerequisited

Ubuntu=22.04. [ROS 2 = Humble](https://docs.ros.org/en/humble/Installation.html). [PCL=1.12](https://pointclouds.org/). [Eigen=3.4](https://eigen.tuxfamily.org/index.php?title=Main_Page)

[GTSAM>=4.2](https://github.com/borglab/gtsam/releases) (the `ros-humble-gtsam` package works)

[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) (only needed for Livox LiDARs)

Additional ROS 2 packages required by this workspace:

```
sudo apt install ros-humble-rviz2 ros-humble-rviz-common ros-humble-rviz-default-plugins \
                 ros-humble-rosbag2-cpp ros-humble-pcl-conversions \
                 ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-eigen \
                 ros-humble-tf2-geometry-msgs ros-humble-tf2-sensor-msgs
```

## 3. Build

```
mkdir -p ~/voxelslam_ws/src
cd ~/voxelslam_ws/src
git clone <this repo>
cd ~/voxelslam_ws
colcon build --symlink-install
source install/setup.bash
```

Both packages in this repository are ROS 2 (ament) packages:

* `voxel_slam` — the SLAM system itself (executable `voxel_slam` / node `voxelslam`).
* `voxelslam_pointcloud2` — the custom RViz2 display plugin that clears the accumulated map when it
  receives an empty point cloud.

## 4. Run Voxel-SLAM

### 4.1 Livox Avia

The online relocalization experiment rosbag. Download: [Onedrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/ErEznhkJzTxJiLuJ8AQDGS0BvCy6KsuaWF2D6cnx061GEQ?e=dMRSlf) ([Google Drive](https://drive.google.com/file/d/1LG46i0vreQrMZRap5tJkjKK4IX0t0zfC/view?usp=drive_link))

```
ros2 launch voxel_slam vxlm_avia.launch.py
// Using the "--pause" guarantees the bag benning time are the same in different runs
// Press the Space to start
rosbag play compus_elevator.bag --pause 
```

In the elevator, the system continues to restart until stepping out of the evevator. The blue point cloud is the map from initialization.

After the rosbag is done, your may find the map is inconsistent as shown in the video. Run

```
rosparam set finish true
```

to launch the final global mapping (global bundle adjustment) to refine the global map.

### 4.2 HILTI 2023 (Multi-Session)

The multi-session experiment rosbag. 

For quick test to download: [Onedrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/Epp5AQ2Oq1VNhC6MIuCAtN4BJC9jx9VvuVx7VT_cdlvD0A?e=8mXYdc). The whole rosbags of HILTI 2022 and 2023 are on the [website](https://hilti-challenge.com/index.html).

The rosbag had better be played from "site1_handheld_5" to "site_handheld_1", or the "site_handheld_2" and "site_handheld_3" cannot find the loop. 

Before launching, please set configure the variables in "hesai.yaml". The '#' means annotation

```
# hesai.yaml
save_path: "${YOUR_FILE_PATH_TO_SAVE_THE_OFFLINE_MAP}"
previous_map: "# site1_handheld_5: 0.50, 
               # site1_handheld_4: 0.45,
               # site1_handheld_3: 0.30,
               # site1_handheld_2: 0.50"
bagname: "site1_handheld_${1-5}" # The rosbag name you play
is_save_map: 1 # Enable to save the map
```

```
ros2 launch voxel_slam vxlm_hesai.launch.py
rosbag play site1_handheld_5.bag --pause
```

```
ros2 launch voxel_slam vxlm_hesai.launch.py // Load the site_handheld_5
rosbag play site1_handheld_4.bag --pause
```

```
ros2 launch voxel_slam vxlm_hesai.launch.py // Load the site_handheld_{5, 4}
rosbag play site1_handheld_3.bag --pause
```

For the "site1_handheld_2", do not forget load the offline maps. The "hesai.yaml" should be like this

```
# hesai.yaml
save_path: "${YOUR_FILE_PATH_TO_SAVE_THE_OFFLINE_MAP}"
previous_map: "site1_handheld_5: 0.50, 
               site1_handheld_4: 0.45,
               site1_handheld_3: 0.30,
               # site1_handheld_2: 0.50"
bagname: "site1_handheld_2" # The rosbag name you play
is_save_map: 1 # Enable to save the map
```

```
ros2 launch voxel_slam vxlm_hesai.launch.py // Load the site_handheld_{5, 4, 3}
rosbag play site1_handheld_2.bag --pause
```

```
ros2 launch voxel_slam vxlm_hesai.launch.py // Load the site_handheld_{5, 4, 3, 2}
rosbag play site1_handheld_1.bag --pause
```

The map may not be consistent as shown in the video. Run

```
rosparam set finish true
```

for the final global BA.

### 4.3 MARS Dataset

For quick test to download: [Onedrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/EpjsGW6coYlMvBWo8TlgJXoBttAuoocLi24V6kw-r_3A8w?e=vVB6RY). The whole rosbags of MARS dataset are on the [website](https://mars.hku.hk/dataset.html).

```
ros2 launch voxel_slam vxlm_avia_fly.launch.py
rosbag play HKisland03.bag --pause
```

The beginning of the point cloud is empty and failing to initialize until the drone at a certain height.

```
ros2 launch voxel_slam vxlm_avia_fly.launch.py
rosbag play AMvalley03.bag --pause
rosparam set finish true
```

This sequence is difficult to find loop. Please run the GBA to ensure the global map consistence.

### 4.4 Livox Mid360

The rosbag begin in a violent speed: [Onedrive](https://1drv.ms/f/c/8b1ef18ae4181c8d/ErtuXCFhFrBErZxzS5vLASkBJEfgDB9R2CSCgKe8BwhneQ?e=zbr7NL)

Livox publishes its own `livox_ros_driver2/msg/CustomMsg` point format rather than
`sensor_msgs/msg/PointCloud2`. This is handled transparently: the bag reader detects the recorded
type and converts it, so a rosbag2 directory can be passed straight in:

```
ros2 launch voxel_slam vxlm_mid360.launch.py \
    bag_path:=/path/to/rosbag2_dir \
    bagname:=jungle_challenge
```

Note that a **ROS 2 (rosbag2) recording** is required — the old ROS 1 `.bag` file must be converted
first (e.g. with `rosbags-convert`). The Livox configs (`mid360.yaml`, `avia.yaml`, `avia_fly.yaml`)
use `lidar_type: 6`; `1..5` are Velodyne/Ouster/Hesai/RoboSense/TartanAir.

### 4.5 Others

Other types of LiDAR will be released later.

## 5. VoxelSLAMPointCloud2

**VoxelSLAMPointCloud2**: A customized plugin for RViz2. It has the same usage to original "PointCloud2" in RViz2, but it can **clear the point cloud map automatically** when receiving an empty point cloud, with any **Decay Time** of the plugin. 

(1) Put the "VoxelSLAMPointCloud2" package within the same "src" folder of your ROS 2 workspace and
`colcon build` it. The package registers itself with `pluginlib` against the `rviz_common`
category, so no manual plugin registration is needed.

(2) `ros2 launch` your program with RViz2 (this repository's launch files already start `rviz2`).

(3) In RViz2 click "Add" and pick "voxelslam_pointcloud2/VoxelSLAMPointCloud2".


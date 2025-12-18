# T-Robotics Dual Steering AMR ROS 2 Software Stack

This repository contains the official ROS 2 software stack for the dual-steering Autonomous Mobile Robot (AMR) platform developed by **T-Robotics**.  
The software was developed as part of the **T-Robotics Industry–University Cooperation Project** at Kyung Hee University.

---

## Overview

This repository provides ROS 2 packages for:

- Kinematics & control of a dual-steering AMR
- 3D LiDAR-based mapping using **Fast-LIO**
- Re-localization on a prior map using Fast-LIO2-Localization
- Autonomous navigation using **Nav2**

The stack is designed for both **simulation** and **real-world deployment**, with a focus on modularity and reusability.

---

## Dependencies

### 1. ROS 2

- **OS**: Ubuntu 22.04
- **ROS 2**: Humble (desktop-full recommended)

> Installation guide: see the official ROS 2 Humble documentation.

---

### 2. ROS 2 Packages (APT)

The following ROS 2 packages are required:

```bash
sudo apt-get update && sudo apt-get install -y \
  # Navigation / Localization
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-teleop-twist-keyboard \
  \
  # Robot description / TF / state
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  \
  # Control
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  \
  # Gazebo (Simulation)
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-plugins \
  \
  # Sensors / Perception
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-diagnostic-updater \
  ros-humble-sick-safetyscanners2
```

---

### 3. External Libraries & Repositories

* **Livox-SDK2**

  * Required for Livox LiDAR devices.
  * Installation: follow the instructions in
    [Livox-SDK2/README.md](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md)

* **livox_ros_driver2**

  * ROS 2 driver for Livox LiDAR.
  * Clone and build in your workspace (e.g., under `~/ros2_ws/src`), or use an existing installation.

* **Fast-LIO / Fast-LIO2-Localization**

  * Used for 3D LiDAR odometry, mapping, and re-localization.
  * Config and launch files referenced in this README assume the directory structure:

    ```bash
    Fast-LIO2-Localization/FAST_LIO/config/...
    Fast-LIO2-Localization/FAST_LIO/launch/...
    ```

---

## Installation

### 1. Create Workspace & Clone

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository (devel branch)
git clone -b devel https://github.com/RCILab/RCI_dual_steering_tr.git
```

### 2. Build

```bash
cd ~/ros2_ws

# 1) Build sensor drivers and simulation packages
colcon build --packages-select livox_ros_driver2 neo_simulation2
source install/setup.bash

# 2) Build Fast-LIO, AMR stack, and ICP-based relocalization
colcon build --packages-select fast_lio tr_dual_steering icp_relocalization
source install/setup.bash
```

> If all required packages are in the same workspace and you prefer a single build:
>
> ```bash
> colcon build
> source install/setup.bash
> ```

---

## Usage

### 1. Simulation

#### 1) Kinematics / Basic Bringup

Start the dual-steering kinematics simulation:

```bash
ros2 launch tr_dual_steering kinematics.launch.py
```

This will bring up the robot model, basic kinematics, and related nodes required for simulation.

---

### 2. Mapping (Fast-LIO)

1. **Run Fast-LIO mapping**

   ```bash
   ros2 launch fast_lio mapping.launch.py
   ```

2. **Drive the robot manually for mapping**

   In a separate terminal:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

   Use the keyboard to move the robot and accumulate a 3D point cloud map.

3. **Save the map (PCD)**

   Once mapping is complete:

   ```bash
   ros2 service call /map_save std_srvs/srv/Trigger "{}"
   ```

   This will save the current accumulated point cloud as a `.pcd` file at the path specified by `map_file_path`.

---

### 3. Mapping Configuration Notes

* `map_file_path` is defined in:

  ```bash
  Fast-LIO2-Localization/FAST_LIO/config/velodyne.yaml
  ```

* For re-localization & prior map usage, you must set:

  1. `prior_map_path` in:

     ```bash
     Fast-LIO2-Localization/FAST_LIO/config/fast_lio_relocalization_param.yaml
     ```

  2. `map_path` in:

     ```bash
     Fast-LIO2-Localization/FAST_LIO/launch/relocalization.launch.py
     ```

  to the same PCD map path (`prior_map_path = map_path = <your_pcd_path>`).

> **Note (KO)**:
>
> * `map_file_path`는 `velodyne.yaml`에서 확인할 수 있으며,
> * Localization 시에는 `fast_lio_relocalization_param.yaml`의 `prior_map_path`와
>   `relocalization.launch.py`의 `map_path`를 생성한 PCD 맵 경로와 동일하게 맞춰줘야 합니다.

---

### 4. 2D Map Generation for Navigation

Before running Nav2, you must convert the 3D PCD map into a 2D occupancy grid (`.pgm + .yaml`) and configure navigation to use it.

1. **Convert PCD → PGM/YAML**

   Use your preferred tool or script (e.g., custom PCD-to-occupancy-grid converter) to generate:

   * `<map_name>.pgm`
   * `<map_name>.yaml`

2. **Update paths in launch files**

   In the following launch files, set your map & PCD paths:

   * `tr_dual_steering/launch/bringup.launch.py`
   * `tr_dual_steering/launch/localization.launch.py`

   Update:

   * `pcd_path` : path to the PCD map (for localization / Fast-LIO re-localization)
   * `map_name` : base name of your 2D map (`.pgm` / `.yaml`)
   * `yaml_path`: full path to the 2D map YAML file

> **Note (KO)**:
> Navigation 전에 PCD 파일을 이용해 2D 지도(pgm & yaml)를 생성해야 하며,
> `bringup.launch.py`와 `localization.launch.py`의 `pcd_path`, `map_name`, `yaml_path`를 자신의 환경에 맞게 수정해야 합니다.

---

### 5. Navigation (Nav2)

After configuring the map and paths:

```bash
ros2 launch fast_lio relocalization.launch.py
ros2 launch tr_dual_steering bringup.launch.py
```

This launch file typically starts:

* Robot description & state publishers
* Localization (e.g., Fast-LIO-based localization + TF tree)
* Nav2 stack (planner, controller, behavior tree, costmaps)
* Any necessary sensor and control interfaces

Once the launch is running:

* Open **RViz2**, load the proper config, and
* Use the **Nav2 panel** to set **2D Pose Estimate** and **2D Goal Pose** for autonomous navigation.

---

## License

This project is licensed under the **MIT License**.
See the [LICENSE](LICENSE) file for details.

---

## Contact

* **Maintainer**: Minjae Jo (조민재) — [alswo0300@khu.ac.kr](mailto:alswo0300@khu.ac.kr)
* **Lab**: [RCI Lab @ Kyung Hee University](https://rcilab.khu.ac.kr)



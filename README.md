# ROS 2 Rolling – Bebop UAV Control Suite

![ROS 2](https://img.shields.io/badge/ROS2-Rolling-blue.svg)
![Python](https://img.shields.io/badge/Python-3.10+-yellow.svg)
![C++](https://img.shields.io/badge/C++-17-orange.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

A full ROS 2 Rolling workspace for controlling and visualizing **Parrot Bebop drones** using both **Python** and **C++** nodes.  
The project integrates computer vision (AprilTag), control loops, trajectory tracking, and live telemetry visualization.

Developed and tested on **Ubuntu 24.04 LTS** with **ROS 2 Rolling**.

---

## 🧩 Repository Overview

This repository contains:

| Package | Description |
|----------|--------------|
| **`nero_drone`** | Core UAV control, AprilTag detection, and reference generation nodes. |
| **`ros2_bebop_driver`** | Driver package for communicating with the Parrot Bebop 1 drone via Wi-Fi. |
| **Utilities** | Visualization and logging scripts for real-time or offline data analysis. |

---

## 🚀 Features

- ✅ Full control loop (velocity, position, yaw).
- ✅ AprilTag detection and TF publishing.
- ✅ Real-time telemetry and trajectory plotting.
- ✅ Configurable reference trajectories.
- ✅ ROS 2 launch and runtime tools for UAV management.

---

## 🛠️ Installation

### 1. Install ROS 2 Rolling

Follow the official guide:  
👉 [ROS 2 Rolling Installation (Ubuntu 22.04)](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html)

Then, source ROS 2:
```bash
source /opt/ros/rolling/setup.bash
```

### 2. Create the Workspace
```bash
mkdir -p ~/ws_uav_ros2/src
cd ~/ws_uav_ros2/src
```

Clone this repository:
```bash
git clone https://github.com/LeonardoFagundesJr/ros2_rolling_bebop_uav.git
```

### 3. Install Dependencies

Install core ROS 2 dependencies:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-rolling-cv-bridge \
  ros-rolling-image-transport \
  ros-rolling-tf2* \
  ros-rolling-geometry-msgs \
  ros-rolling-sensor-msgs \
  ros-rolling-visualization-msgs \
  ros-rolling-nav-msgs
```


Initialize ```rosdep```:

```bash
cd ~/ws_uav_ros2
sudo rosdep init   # skip if already done
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```


### 4. Build the Workspace

```bash
cd ~/ws_uav_ros2
colcon build --symlink-install
```

After successful build:
```bash
source install/setup.bash
```

To make it permanent, add to your ```.bashrc```:
```bash
echo "source ~/ws_uav_ros2/install/setup.bash" >> ~/.bashrc
```


## 🎮 Running the System
### 1. Launch the Bebop driver

Connect to your Bebop drone’s Wi-Fi (default IP: ```192.168.42.1```).

```bash
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=192.168.42.1 namespace:=B1
```
This starts the ROS 2 node that communicates with the drone.

### 2. Launch the full UAV control system

```bash
ros2 launch nero_drone full_bebop.launch.py
```

Optionally, you can use:
```bash
ros2 launch nero_drone full_bebop.launch.py onlycontrol:=true
```
to launch only the control subsystem without visualization.

### 3. Run the velocity logger

```bash
ros2 run nero_drone velocity_logger
```
Logs drone velocity data to topics and saves it for later analysis.

### 4. Publish a reference trajectory

```bash
ros2 run nero_drone ref_pos.py
```
This node sends a reference vector of 8 parameters:

| Field            | Meaning                       |
| :--------------- | :---------------------------- |
| `x`, `y`, `z`    | Desired position (meters)     |
| `yaw`            | Desired orientation (radians) |
| `dx`, `dy`, `dz` | Velocity components (m/s)     |
| `dyaw`           | Angular velocity (rad/s)      |

### 5. Visualize logged data (offline)
Inside the Python scripts directory:
```bash
cd ~/ws_uav_ros2/src/nero_drone/nero_drone
python3 graphical.py
```
This script plots the *entire experiment* using stored log data.


### 6. Real-time visualization
To monitor telemetry and trajectories live:
```bash
cd ~/ws_uav_ros2/src/nero_drone/nero_drone
python3 graphs.py
```
Displays live plots of position, velocity, and control signals.

## 🧠 Project Structure

```
ws_uav_ros2/
├── src/
│   ├── nero_drone/
│   │   ├── src/               # C++ nodes
│   │   ├── nero_drone/        # Python nodes & tools
│   │   ├── launch/            # ROS 2 launch files
│   │   ├── config/            # YAML configuration files
│   │   ├── urdf/, meshes/, others/
│   │   └── CMakeLists.txt
│   └── ros2_bebop_driver/
└── install/
```

## 🧩 Key Nodes Overview

| Node                               | Description                                            |
| ---------------------------------- | ------------------------------------------------------ |
| `Joy2Cmd`                          | Converts joystick input into UAV command messages.     |
| `beboptag`                         | Detects AprilTags and computes camera pose.            |
| `tf_tag_bebop`                     | Publishes TF transforms from tags to drone base frame. |
| `RefPublisher`                     | Publishes reference positions and orientation.         |
| `velocity_logger`                  | Records velocity data for analysis.                    |
| `ref_vec_filter`                   | Filters noisy reference data.                          |
| `tagfollow`                        | Implements autonomous tag following logic.             |
| `visual`                           | Visual visualization and TF broadcaster.               |
| `land`, `isfly`, `safety_watchdog` | Basic safety and state monitoring nodes.               |

## ⚙️ Configuration
All configurable parameters (PID gains, topics, camera settings, etc.) are located in:
```bash
nero_drone/config/
```
and can be modified before launch.


## 🧾 License
This project is licensed under the MIT License.

## 🤝 Contributing

Pull requests are welcome!
To contribute:

1. Fork this repository.
2. Create a feature branch (```git checkout -b feature/your-feature```).
3. Commit changes: ```git commit -m "Add new feature"```
5. Push: ```git push origin feature/my-feature```
6. Open a Pull Request 🚀

<!-- ## 📬 Contact -->
<!-- Author: Leonardo Fagundes Jr
📧 Email: (add your email if you’d like)
🌐 GitHub: LeonardoFagundesJr -->


Built with ❤️ using ROS 2 Rolling, OpenCV, and Eigen for robust aerial robotics research.


<!-- 
```bash


``` -->


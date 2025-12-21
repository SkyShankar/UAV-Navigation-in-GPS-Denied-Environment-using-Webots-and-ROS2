# UAV Navigation in GPS-Denied Environment using Webots and ROS 2

This repository contains the simulation framework and source code for the Master's Thesis: **"Cooperative UAV Navigation in GPS-Denied Environments Using ROS2 and Webots Simulation"**

The project implements a **Leader-Follower** architecture where a follower drone(No GPS) autonomously tracks and follows a moving leader drone(GPS) without relying on Global Positioning Data. Instead, it fuses onboard Inertial Measurement Unit (IMU) data with relative visual estimates (Camera (YOLO) + LiDAR) to perform cooperative localization and nonlinear optimal control.

## 🚀 Key Features

* **Simulation Environment:** Developed a high-fidelity physics simulation using Webots and ROS 2 (Humble) middleware to model complex drone dynamics.
* **Computer Vision:** Integrated YOLOv8 for real-time object detection to robustly identify and track the leader drone within the camera frame.
* **State Estimator:** Designed and implemented custom Extended Kalman Filter (EKF) and State-Dependent Riccati Equation (SDRE) filters to fuse high-rate IMU data with low-rate relative position measurements.
* **Sensor Fusion:** Implemented multi-sensor fusion (Camera/YOLOv8 + LiDAR) to estimate the leader’s relative position within the follower's local coordinate system, enabling precise distance maintenance (Formation Flying).
* **Cooperative Localization and Navigation:** Validated the framework in two distinct operational scenarios:
*    **Scenario 1:** Active V2V communication where the Leader transmits its GPS coordinates. The follower computes a "Pseudo-GPS" position by combining relative sensor data with the leader's global position.
*    **Scenatio 2:** Operates without communication or global references. The follower maintains formation solely relying on onboard sensor fusion for relative navigation, validating autonomy in GPS-denied environments.

## 📊 Results

The framework was tested in scenarios involving curved paths and continuous turning.
* **Tracking Accuracy:** Achieved precise leader-follower formation by maintaining a fixed relative distance via robust Vision & LiDAR sensor fusion.
* **Disturbance Rejection:** Validated EKF and SDRE filters, demonstrating effective active noise rejection and stable state estimation during flight.
* **Scenario 1 vs. Scenario 2:** Benchmarked performance between global localization (Pseudo-GPS, Scenario 1) and a fully GPS-denied environment (Scenario 2). The system successfully maintained formation in Scenario 2 solely through local relative localization, proving the robustness of the sensor fusion algorithm.

---

## 📂 Repository Structure

The project is organized as a ROS 2 workspace:

* **`src/mavic2_webots/`**: Contains the simulation assets.
    * `worlds/`: The simulation environments.
    * `controllers/`: Webots controllers.
    * `protos/`: Custom drone models.
* **`src/mavic2_ekf_pkg/`**: The core ROS 2 package.
    * `launch/`: Launch files to start the simulation and nodes.
    * `mavic2_ekf_pkg/`: Python source code for nodes (Logger, EKF, etc.).

## 🛠️ Prerequisites

* **OS:** Ubuntu 22.04.5 LTS
* **ROS 2:** Humble Hawksbill
* **Webots:** R2024b (or newer)
* **ros-webots-bridge:** `ros-humble-webots-ros2`

## ⚙️ Installation

### 1. Install Webots
The recommended way is to use the official Cyberbotics APT repository. Run the following commands:

```bash
# Add the Cyberbotics repository
sudo mkdir -p /etc/apt/keyrings
cd /etc/apt/keyrings
sudo wget -q [https://cyberbotics.com/Cyberbotics.asc](https://cyberbotics.com/Cyberbotics.asc)
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] [https://cyberbotics.com/debian](https://cyberbotics.com/debian) binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list

# Update and install
sudo apt update
sudo apt install webots
```

### 2. **Install ROS 2 Humble**
```bash
# Set locale
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup Sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL [https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos](https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos) -o /tmp/ros2.repos # This line is often handled by adding the key manually, simpler version below:

# Authorize ROS 2 GPG Key
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

### 3.  **Install Webots-ROS2 Bridge**
```bash
sudo apt install ros-humble-webots-ros2
```

### 4.  **Clone the repository:**
```bash
git clone [https://github.com/SkyShankar/UAV-Navigation-in-GPS-Denied-Environment-using-Webots-and-ROS2.git](https://github.com/SkyShankar/UAV-Navigation-in-GPS-Denied-Environment-using-Webots-and-ROS2.git)
```

### 5.  **Navigate to the workspace:**
```bash
cd UAV-Navigation-in-GPS-Denied-Environment-using-Webots-and-ROS2
```

### 6.  **Install dependencies for ROS2 and YOLOv8:**
```bash
sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src -r -y

pip3 install ultralytics # YOLOv8 Library
```

### 7.  **Build the workspace:**
```bash
colcon build --symlink-install
```

### 8.  **Source the setup file:**
```bash
source install/setup.bash
```

## ▶️ Usage

To launch the simulation and the EKF nodes:

```bash
ros2 launch mavic2_ekf_pkg <your_launch_file>.py



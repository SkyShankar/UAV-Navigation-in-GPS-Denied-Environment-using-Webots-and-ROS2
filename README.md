# UAV Navigation in GPS-Denied Environment using Webots and ROS 2

This repository contains a simulation framework for autonomous UAV navigation in GPS-denied environments. It utilizes **Webots** for physics simulation and **ROS 2** for control logic, state estimation, and sensor fusion.

## 🚀 Project Overview

The primary goal is to estimate the state (position, velocity, orientation) of a Mavic 2 Pro drone without relying on GPS data. The system uses an **Extended Kalman Filter (EKF)** to fuse data from onboard sensors (IMU, Optical Flow, etc.).

### Key Features
* **Simulation:** High-fidelity Mavic 2 Pro simulation in Webots.
* **State Estimation:** Custom EKF implementation specifically for GPS-denied navigation.
* **ROS 2 Architecture:** Modular design with separate nodes for logging, filtering, and control.
* **Data Analysis:** Integrated logging tools to generate performance graphs (`logs/` folder).

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

### 6.  **Install dependencies:**
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
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



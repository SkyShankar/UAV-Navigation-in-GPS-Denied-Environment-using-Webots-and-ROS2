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

2.  **Clone the repository:**
    ```bash
    git clone [https://github.com/SkyShankar/UAV-Navigation-in-GPS-Denied-Environment-using-Webots-and-ROS2.git](https://github.com/SkyShankar/UAV-Navigation-in-GPS-Denied-Environment-using-Webots-and-ROS2.git)
    ```

3.  **Navigate to the workspace:**
    ```bash
    cd UAV-Navigation-in-GPS-Denied-Environment-using-Webots-and-ROS2
    ```

4.  **Install dependencies:**
    ```bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```

5.  **Build the workspace:**
    ```bash
    colcon build --symlink-install
    ```

6.  **Source the setup file:**
    ```bash
    source install/setup.bash
    ```

## ▶️ Usage

To launch the simulation and the EKF nodes:

```bash
ros2 launch mavic2_ekf_pkg <your_launch_file>.py




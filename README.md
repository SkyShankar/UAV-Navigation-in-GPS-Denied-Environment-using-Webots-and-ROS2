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
    * `controllers/`: Low-level Webots controllers.
    * `protos/`: Custom drone models.
* **`src/mavic2_ekf_pkg/`**: The core ROS 2 package.
    * `launch/`: Launch files to start the simulation and nodes.
    * `mavic2_ekf_pkg/`: Python source code for nodes (Logger, EKF, etc.).

## 🛠️ Prerequisites

* **OS:** Ubuntu 20.04 (Foxy) or 22.04 (Humble)
* **ROS 2:** Foxy/Humble (depending on your system)
* **Webots:** R2023b (or your specific version)
* **ros-webots-bridge:** `sudo apt install ros-<distro>-webots-ros2`

## ⚙️ Installation

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/SkyShankar/UAV-Navigation-in-GPS-Denied-Environment-using-Webots-and-ROS2.git](https://github.com/SkyShankar/UAV-Navigation-in-GPS-Denied-Environment-using-Webots-and-ROS2.git)
    ```

2.  **Navigate to the workspace:**
    ```bash
    cd UAV-Navigation-in-GPS-Denied-Environment-using-Webots-and-ROS2
    ```

3.  **Install dependencies:**
    ```bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```

4.  **Build the workspace:**
    ```bash
    colcon build --symlink-install
    ```

5.  **Source the setup file:**
    ```bash
    source install/setup.bash
    ```

## ▶️ Usage

To launch the simulation and the EKF nodes:

```bash
ros2 launch mavic2_ekf_pkg <your_launch_file>.py

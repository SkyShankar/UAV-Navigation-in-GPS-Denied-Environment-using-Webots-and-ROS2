# UAV Navigation in GPS-Denied Environment using Webots and ROS 2

This repository contains the simulation framework and source code for the Master's Thesis: **"Robust State Estimation and Control for UAV Formation Flight using Vision-LiDAR Odometry."**

The project implements a **Leader-Follower** architecture where a follower drone autonomously tracks a moving leader drone without relying on its own Global Positioning System (GPS). Instead, it fuses onboard Inertial Measurement Unit (IMU) data with relative visual estimates (YOLO + LiDAR) to perform cooperative localization and nonlinear optimal control.

## 🚀 Key Features

* **Simulation Environment:** High-fidelity physics simulation using **Webots** with **ROS 2 (Humble)** middleware.
* **Computer Vision:** Real-time object detection using **YOLOv11** to identify the leader drone in the camera frame.
* **Sensor Fusion:** A custom **Extended Kalman Filter (EKF)** that fuses high-rate IMU data with low-rate relative position measurements.
* **Nonlinear Control:** Implementation of the **State-Dependent Riccati Equation (SDRE)** controller for aggressive maneuver tracking, handling nonlinearities like gravity coupling.
* **Cooperative Localization:** The follower estimates its global state by transforming relative body-frame measurements into the navigation frame using derived kinematic relationships.

---

## 🛠️ System Architecture

The system consists of two drones: a **Leader** (which follows a pre-defined path) and a **Follower** (which must maintain a fixed distance and heading).

### 1. Sensing & Perception
The follower drone is equipped with:
* **RGB Camera:** Captures images for the YOLOv11 model to calculate the bounding box center $(u, v)$ of the leader.
* **LiDAR:** Provides the scalar distance ($D$) to the leader.
* **IMU:** Provides angular rates $(p, q, r)$ and linear accelerations $(a_x, a_y, a_z)$.

### 2. Relative Localization Strategy
Instead of relying on GPS, the follower calculates the relative vector to the leader in 3D space:
1.  **Visual Servoing:** The horizontal ($\alpha$) and vertical ($\beta$) bearing angles are derived from the camera's Field of View (FOV) and the target's pixel coordinates.
2.  **Spherical-to-Cartesian:** The scalar distance $D$ from LiDAR is projected using $\alpha$ and $\beta$ to obtain the relative position vector in the **Body Frame**:
    $$\vec{P}_{body} = [d_x, d_y, d_z]^T$$
3.  **Frame Transformation:** To utilize this in the EKF, the vector is transformed to the **Navigation Frame** (ENU) using the transpose of the rotation matrix $C_{bn}$ (Navigation-to-Body):
    $$\vec{P}_{nav} = C_{bn}^T \cdot \vec{P}_{body}$$

### 3. State Estimation (EKF)
An **Extended Kalman Filter** estimates the 9-DOF state vector:
$$\mathbf{x} = [x, y, z, U, V, W, \phi, \theta, \psi]^T$$

* **Prediction:** Standard kinematic propagation using IMU inputs.
* **Correction:** Uses the transformed relative position combined with the leader's known position to generate "Pseudo-GPS" updates.
* **Gravity Regularization:** Special care is taken in the system matrix $\mathbf{A}(\mathbf{x})$ to regularize singular gravity terms (e.g., $\frac{\sin\theta}{\theta}$) during hover conditions.

### 4. Control (SDRE)
The control allocation uses the **State-Dependent Riccati Equation (SDRE)** method. Unlike standard PID, SDRE treats the system matrices as state-dependent ($\mathbf{A}(\mathbf{x})$), allowing for optimal control over the full flight envelope.
* **Guidance:** A visual servoing loop commands **Yaw** to center the leader ($\alpha \to 0$).
* **Stability:** Inner-loop SDRE controllers maintain stability by fighting "parasitic" roll disturbances induced by continuous yawing maneuvers.

---

## 📊 Results

The framework was tested in scenarios involving curved paths and continuous turning.
* **Tracking Accuracy:** The system successfully maintains the leader within the camera FOV using the visual servoing yaw controller.
* **Disturbance Rejection:** The controller effectively compensates for the roll coupling ($\approx -0.2$ rad) generated during sustained yaw maneuvers, preventing lateral drift.
* **Estimation:** The EKF provides smooth state estimates even when visual measurements are noisy or intermittent.

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



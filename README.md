# Distributed Autonomous Exploration and Mapping

This project implements a distributed ROS 2 (Jazzy Jalisco) system for the Thymio II robot. It performs Simultaneous Localization and Mapping (SLAM) and autonomous navigation with obstacle avoidance using a LiDAR sensor.

The architecture splits the computational load and power consumption:
* **Raspberry Pi :** Handles the LiDAR driver and network communication inside a Docker container.
* **PC / Workstation :** Handles the Thymio control (via USB Dongle), SLAM algorithms, Odometry calculation, and Visualization (RViz).

## Hardware Architecture

To prevent power brownouts on the Raspberry Pi, the hardware must be connected as follows:

| Component | Connected To | Connection Type | Description |
| :--- | :--- | :--- | :--- |
| **LD06 LiDAR** | **Raspberry Pi** | USB (UART) | Publishes `/scan` data over Wi-Fi. |
| **Thymio Wireless Dongle** | **PC (Ubuntu)** | USB | Controls motors and reads robot sensors. |
| **Thymio Robot** | Wireless | RF (via Dongle) | Receives motor commands, sends sensor data. |

---

##  Prerequisites

### 1. PC 
* **OS:** Ubuntu 24.04 LTS (Noble Numbat).
* **ROS 2:** Jazzy Jalisco.
* **Dependencies:** `ros-jazzy-slam-toolbox`, `ros-jazzy-navigation2`, `ros-jazzy-rmw-cyclonedds-cpp`.
* **Python Libs:** `thymiodirect`, `flask`.

### 2. Raspberry Pi 
* **OS:** Raspberry Pi OS (64-bit) / Debian Bookworm.
* **Software:** Docker & Docker Compose.
* **Network:** Must be on the same Wi-Fi network as the PC.

---

## Installation

### 1. Prerequisites
* **PC:** Ubuntu 24.04 / ROS 2 Jazzy / slam_toolbox.
	* https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
	* https://github.com/SteveMacenski/slam_toolbox
* **Raspberry Pi:** Raspberry Pi OS / Ubuntu (Docker recommended).
* **Python Libs:**  thymiodirect, flask,  serial.

### 2. Setup 

#### Part A: PC

1.  **Clone repository:**
```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone <GIT_URL> .
```


2.  **Build the Workspace:**
  ```bash
    cd ~/mb_project
    colcon build --symlink-install
    source install/setup.bash
    ```

#### Part B: Raspberry Pi Setup (Docker)

1.  **Transfer the source code to the Pi**.

2.  **Build the Docker Image:**
    Navigate to the folder containing the `Dockerfile` and run:
```bash
    docker build -t thymio_bot_img .
```

---

##  Network Configuration 

Since the system uses Wi-Fi, we use **DDS** to manage ROS 2 communication efficiently.

1.  **Export Environment Variable:**
    Before running any ROS command, run these lines (or add them to `.bashrc`):
```bash
    #just do this on pc because is already set on Dockerfile for pi
    export ROS_DOMAIN_ID=42
```

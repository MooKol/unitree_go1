# Unitree GO1 Robot ROS 2

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-green)
![distro](https://img.shields.io/badge/ROS2-Humble-blue)
[![humble](https://github.com/IntelligentRoboticsLabs/go2_robot/actions/workflows/humble.yaml/badge.svg)](https://github.com/IntelligentRoboticsLabs/go2_robot/actions/workflows/humble.yaml)


In this package is our integration for the Unitee Go1 robot.

## Checklist

- [x] robot description
- [x] odom
- [x] pointcloud
- [x] joint_states
- [x] Visualization in rviz
- [x] cmd_vel
- [x] go1_interfaces
- [x] Change modes
- [x] Change configuration for robot
- [x] SLAM
- [x] Nav2
- [x] Gazebo simulation

## Installation
You need to have previously installed ROS2. Please follow this [guide](https://docs.ros.org/en/humble/Installation.html) if you don't have it.

```bash
source /opt/ros/humble/setup.bash
```

## 1. Installation

### 1.0 Install ROS-based dependencies:
```bash
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-velodyne
sudo apt install ros-humble-velodyne-gazebo-plugins
sudo apt-get install ros-humble-velodyne-description
```

### 1.1 Clone and install all dependencies:
    
```bash
sudo apt install -y python3-rosdep
rosdep update
git clone https://github.com/MooKol/unitree_go1.git
cd <sim>
rosdep install --from-paths src --ignore-src -r -y
```

Prepare your thirparty repos:
```bash
sudo apt update && sudo apt install ros-dev-tools -y
vcs import < go1_robot/dependencies.repos
```

Install dependencies and build workspace
```bash
cd ~/go2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build 
```

Setup the workspace
```bash
source install/setup.bash
```

## Simulation in Gazebo
To test the autonomous navigation of the Go1 robot in the simulation environment(Gazebo), follow the steps provided in this repository.

## Simulation Demo

<p align="center">
  <img src="docs/Media3.gif" width="800" alt="Unitree Go1 Simulation Demo">
</p>

```bash
cd ~/sim
colcon build
source install/setup.bash
ros2 launch go1_config gazebo.launch.py 
```

Open a new terminal

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
Now you are able to move the Go1 with the keyboard. if you want to move the robot Autonamously, follow these step:

Open a new terminal

```bash
source install/setup.bash
ros2 launch go1_config slam.launch.py restart_map:=true use_rviz:=false
```

Open a new terminal

```bash
source install/setup.bash
ros2 launch go1_config navigate.launch.py 
```

## Real World Experiments
To test the autonomous navigation of the Go1 robot in a real-world environment, follow the steps provided in this repository.

## Navigation Demo
<p align="center">
  <img src="docs/Media1.gif" width="800" alt="Unitree Go1 Navigation Demo">
</p>


## Inspection Demo
<p align="center">
  <img src="docs/Media2.gif" width="800" alt="Unitree Go1 Inspection Demo">
</p>



## Citations

If you use this work or our data in your research, please cite it appropriately. The following BibTeX entrie is the most accurate versions available:


Petropoulakis, P. and Kolani, M R. and Borrmann, A.: CAutonomous operation of a robot dog for point-cloud data acquisition. The 32nd EG-ICE International Workshop on Intelligent Computing in Engineering (EG-ICE), 2025

    @inproceedings{GO2:2025:EGICE, 
        title={Autonomous operation of a robot dog for point-cloud data acquisition}, 
        author={Petropoulakis, P. and Kolani, M R. and Borrmann, A.}, 
        year={2025}, 
        month={May}, 
        yearmonth={2025-05}, 
        keywords={LOCenter; GNI; GNI; }, 
        url={https://publications.cms.ed.tum.de/2024_Kolani_myCon.pdf}, 
        booktitle={The 32nd EG-ICE International Workshop on Intelligent Computing in Engineering}, 
        location={Glasgow, Scotland}, 
    }


Thank you for acknowledging our work!
**CSHS Submarine: Cairns State High School Submarine**
=====================================

**Table of Contents**
-------------------

1. [Description](#description)
2. [Hardware Requirements](#hardware-requirements)
3. [Installation](#installation)
4. [Hardware Interface](#hardware-interface)
5. [TODO](#todo)

**Description**
---------------

The CSHS Submarine is an under water ROV designed as a modular base for underwater exploration and research. It is built around a Raspberry Pi running Ubuntu 22.04, ROS2 Humble, and ROS2 Control. The submarine uses 6 motors controlled by an esp32 connected to the raspberry pi running ros2.

**Hardware Requirements**
------------------------

* Raspberry Pi (running Ubuntu 22.04)
* ESP32

**Installation**
---------------

### ROS2 Humble Installation

Follow the official ROS2 installation guide for Ubuntu 22.04: https://index.ros.org/doc/ros2/Installation/Linux-Install-ROS-2/

### Clone and Run the Code

1. Clone the repository:
```
   git clone https://github.com/TheHacker3256/CSHS_Submarine.git sub
```
2. Navigate to the repository:
```
   cd sub
```
3. Install the required dependencies:
```
   rosdep install -i --from-paths package --rosdistro humble
```
4. Run the car (Real Bot):
```
  ros2 launch sub_rsp launch_bot.launch.py
```
4. Run the car (Simulation):
```
  ros2 launch sub_rsp launch_sim.launch.py
```



**Hardware Interface**
---------------------

The car uses an ESP32 microcontroller to interface with the 6 Electronic speed controllers for the motrs. The ESP32 is configured to talk to the Raspberry Pi over serial using the esp32_comm package: https://github.com/TheHacker3256/esp32_comm.git (The 6 motor branch)


**TODO**
---------
- [ ] Make a Dockerfile for easy launch and install
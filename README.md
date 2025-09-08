# AUV Docking Station Occupancy Sensor System

## Description

### Overview

This repository contains code written for an occupancy sensor system intended to detect the presence of an AUV in the carriage of a docking station called SeaCube. The system consists of four sensing methods (limit switch, RFID reader, induction, and an IMU) and two microcontrollers (Arduino Uno, Teensy 4.1), which communicate with the Raspberry Pi (RPi) computer controlling the docking station’s systems.

For information regarding electrical connections, physical implementation, and each sensor’s performance, the following paper authored for this system should be consulted: https://conservancy.umn.edu/items/e0c4c938-1d60-4180-b31d-45353bf069d8 (also available by searching *An Evaluation of Occupancy Sensing Methods for Autonomous Underwater Vehicle Docking* on the University of Minnesota Digital Conservancy).

The following highlights relevant directories and files:

```
📂microcontroller_sketches
 ├──seacube_arduino_uno.ino      # Arduino Uno firmware
 └──seacube_teensy.ino           # Teensy firmware
📂occupancy_sensor              # ROS package
 ├──📂launch
 │   └──occupancy.launch         # ROS launch file
 ├──📂src
 │   ├──occupancy_pub_node.py    # ROS publisher node
 │   └──occupancy_sub_node.py    # ROS subscriber node
```
The following describes the flow of sensor data:
`Sensors –> Teensy –> RPi ROS Publisher –> RPi ROS Subscriber`.
The ROS publisher parses the sensor data input from the Teensy, and the ROS subscriber processes the parsed data and responds accordingly.

**IMPORTANT:**
Note that the Arduino Uno simply acts as a middleman to relay the RFID reader’s output to the Teensy. Originally, this was done because polling the RFID reader took a significant amount of time, which conflicted with the Teensy’s 200 Hz loop rate. However, it was later discovered that the RFID reader features an interrupt pin that outputs a low digital signal when an RFID tag is detected. Polling this pin rather than the PN532 chip would eliminate the need for the Uno. I recommend this approach, as it simplifies the system (note that even if the Arduino Uno shield variant of the PN532 RFID reader is purchased, it should still be controllable using a Teensy).

### Firmware Notes

See the comments at the top of each .ino file for important functions. Additional notes are as follows:

- `seacube_arduino_uno.ino` sends a digital signal to the Teensy to indicate alignment between the RFID reader on SeaCube’s carriage and the RFID tag within the AUV
- `seacube_arduino_uno.ino` lines 28-31, 63, and 67 can be commented out when not debugging
- `seacube_teensy.ino` performs a high-pass filtering algorithm at 200 Hz on the Y acceleration vector from the accelerometer to detect the abrupt acceleration changes of collisions between the AUV and the station. The IMU must be oriented such that the Y acceleration vector is in the direction of the AUV's approach when docking
- `seacube_teensy.ino` must perform this 200 Hz accelerometer processing rather than the subscriber node, since ROS topic publishing is too slow to transfer data at 200 Hz
- `seacube_teensy.ino` outputs sensor data to RPi every 8th iteration of its 200 Hz loop, resulting in a 25 Hz data publishing rate from the Teensy to the RPi
- `seacube_teensy.ino` might not run `loop()` at exactly 200 Hz, as it is subject to some error depending on the exact time taken to transmit data to the RPi
- `seacube_teensy.ino` should not enable any IMU features other than line 73
- `seacube_teensy.ino` uses a `micros()` function to apply delays to maintain a 200 Hz loop rate. After approximately 70 minutes, the function will overflow and reset back to zero. This was not accounted for when writing this file. This should be addressed before long-term deployments of the system
- Similarly, `seacube_arduino_uno.ino` has a `millis()` function whose overflow/reset is not accounted for

### ROS Software Notes

See the comments left throughout the publisher and subscriber node .py files. Additional notes are as follows:

- The `occupancy_sensor` directory is a ROS package containing all the system’s software run by the RPi
- `occupancy_pub_node.py` parses sensor data input from Teensy and publishes this data to the subscriber node for processing at 25 Hz via these topics:
	- `switch_topic`
	- `rfid_topic`
	- `charging_topic`
	- `filter_topic`
- Note that `imu_topic` is not used by the subscriber node. This topic is intended for other SeaCube processes not related to occupancy detection
- `occupancy_sub_node.py` performs averaging of the INA169 analog output to account for signal noise
- `occupancy_sub_node.py` holds `self.filter_occupancy` true for 5 seconds following a collision, as it is unlikely that the AUV will initially collide in the correct orientation. This allows the AUV time to trigger another sensor. The simultaneous setting of both the `filter_occupancy` boolean and an additional sensor’s `*_occupancy` boolean indicates occupancy with higher confidence.
- `occupancy_sub_node.py` publishes to two topics solely for testing/bagging. The first is `occupancy_topic`, which transfers strings listing which of the four sensing methods have been triggered. The second is `average_INA169_topic`, which transfers the averaged INA169 analog readings. These topics may be removed if desired. If removed, these topics should also be removed from the rosbag command in `occupancy.launch`
- `occupancy.launch` is an executable launch file that launches the publisher node and subscriber node. The `actuator_node` on line 4 controls SeaCube’s carriage motor
- `occupancy.launch` lines 7-8 can be uncommented to run an optional rosbag to bag data while running the system for later analysis 

## Installation/Setup

### General Setup

Flash the Teensy with `seacube_teensy.ino` and, if used, the Arduino Uno with `seacube_arduino_uno.ino`. 

The ROS nodes must be run on the Noetic distribution of the Robot Operating System (ROS). SeaCube’s RPi accomplishes this using a Docker container. Once ROS Noetic is available, copy the contents of this repository’s `occupancy_sensor` ROS package to the `catkin_ws/src/` directory.

To remotely control the RPi while SeaCube was deployed, a TP-Link router was placed within the docking station’s housing. Connecting a laptop running PuTTY and the RPi to the router’s network enabled a Secure Shell (SSH) connection. Alternatively, a Wi-Fi hotspot can be used for the network.

Ensure the following USB connections are in place:
- If used, the Arduino Uno may be powered by any USB source, so long as it shares a ground with the RPi
- The Teensy must be connected to a RPi USB port for both power and serial communication. Take note of the port used (e.g. `/dev/ttyACM1`)

### Repository Customizations

This repository has a number of code segments that are specific to SeaCube and should be removed or modified to fit another system. These include:
- `occupancy_pub_node.py` line 112 must be replaced with the RPi USB port connected to the Teensy
- `imu_topic` and associated variables (e.g. `imu_msg`) can be removed from `occupancy_sub_node.py`, as the data published over this topic is intended for other SeaCube processes not related to occupancy detection
- Similarly, `seacube_teensy.ino`’s `buildOutputString()` function can be modified to no longer transmit the X, Y, and Z acceleration vectors to the RPi. If removed, the parsing performed in `occupancy_pub_node.py` must be modified.
- `occupancy_sub_node.py` lines 28-36 and 43-44 specify SeaCube-specific items and can be removed
- `occupancy_sub_node.py`’s `actuate()` method should be rewritten with the desired conditions that should set `occupancy_status` and the desired response to detecting AUV occupancy
- `occupancy.launch` line 4 is SeaCube specific and can be removed

Additionally, it may be necessary to calibrate/modify the following threshold values:
- `occupancy_sub_node.py`’s `self.filter_threshold` variable represents the threshold that the high-pass filter output must surpass to indicate a collision. It must be adjusted depending on environmental factors (e.g. wave intensity) and the force at which the AUV will collide with the station when docking
- `occupancy_sub_node.py`’s `self.charging_threshold` variable represents the threshold that the averaged INA169 analog output must surpass to indicate alignment of the inductive coils. The analog value is proportional to the current circulating the inductive transmitting coil, which is dependent on the medium the coil is suspended in (see the system’s paper for more information).

## Usage

- Ensure each sensor’s electrical connections are in place as described in the linked paper
- Ensure the USB connections are in place as described in Installation/Setup
- If controlling the RPi remotely, establish an SSH connection
- If Docker is used, enter the Docker container running ROS Noetic 
- `source setup.bash` from `catkin_ws/devel`
- `sudo chmod 666 /dev/ttyACM1` to allow USB read/write permissions (replace `ttyACM1` with the RPi USB port connected to the Teensy) 
- `export ROS_MASTER_URI=http://127.0.0.1:11311`
- `export ROS_HOSTNAME=127.0.0.1`
- Determine whether the data will be bagged by either commenting or uncommenting the rosbag command in `occupancy.launch`
- `roslaunch occupancy_sensor occupancy.launch` to run the launch file and start the system

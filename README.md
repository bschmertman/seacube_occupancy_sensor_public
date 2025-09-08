# AUV Docking Station Occupancy Sensor System

## Description

### Overview

This repository contains code written for an occupancy sensor system intended to detect the presence of an AUV in the carriage of a docking station called SeaCube. The system consists of 
four sensing methods (limit switch, RFID reader, inductive transmitter/INA169 current sensor, and an IMU) and two microcontrollers (Arduino Uno, Teensy 4.1) which communicate with the Raspberry Pi (RPi) computer that controls the docking station.

For information regarding electrical connections and physical implementation, the following paper authored for this system should be consulted: https://conservancy.umn.edu/items/e0c4c938-1d60-4180-b31d-45353bf069d8 (also available by searching *An Evaluation of Occupancy Sensing Methods for Autonomous Underwater Vehicle Docking* on the University of Minnesota Digital Conservancy).

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
`Sensor Data → Teensy → RPi ROS Publisher → RPi ROS Subscriber`.
The ROS publisher parses the sensor data input from the Teensy, and the ROS subscriber processes the parsed data and responds accordingly.

**IMPORTANT:**
Note that the Arduino Uno simply acts as a middleman to relay the RFID reader’s response to the Teensy. Originally, this was done because polling the RFID reader took a significant amount of time, which conflicted with the Teensy’s 200 Hz loop rate. However, it was later discovered that the RFID reader features an interrupt pin which outputs a low digital signal when the presence of an RFID tag is detected. Polling this pin rather than the PN532 chip would eliminate the need for the Uno. I recommend this approach, as it simplifies the system (even if the Arduino Uno shield variant of the PN532 RFID reader is purchased, it should still be controllable using a Teensy).

### Firmware Notes

See the comments at the top of each .ino file for important functions. Additional notes are as follows:

- `seacube_arduino_uno.ino` sends a digital signal to the Teensy to indicate alignment between the RFID reader on SeaCube’s carriage and the the RFID tag within the AUV
- `seacube_arduino_uno.ino` lines 28-31 can be commented out when not debugging
- `seacube_teensy.ino` performs a high-pass filtering algorithm at 200 Hz on the raw accelerometer output to detect the abrupt acceleration changes of collisions between the AUV and station
- `seacube_teensy.ino` must execute this 200 Hz accelerometer processing, since ROS topic publishing is too slow to operate at this rate
- `seacube_teensy.ino` outputs sensor data to RPi every 8th iteration of its 200 Hz loop (25 Hz)
- `seacube_teensy.ino` may not run `loop()` at exactly 200 Hz, it is subject to some error depending on the exact time taken to transmit data to the RPi


### ROS Software Notes

See the comments left throughout the publisher and subscriber node .py files. Additional notes are as follows:

- `occupancy_sensor` directory is a ROS package containing all the system’s software run by the RPi
- `occupancy_pub_node.py` parses sensor data input from Teensy and publishes this data to the subscriber node for processing at 25 Hz via these 5 topics:
	- `switch_topic`
	- `rfid_topic`
	- `charging_topic`
	- `imu_topic`
	- `filter_topic`
- `occupancy_sub_node.py` performs averaging of the INA169 analog output to account for noise
- `occupancy_sub_node.py` holds self.filter_occupancy true for 5 seconds following a collision since it is unlikely that the AUV will initially collide in the correct orientation. This allows the AUV time to trigger another sensor. The simultaneous setting of both the filter boolean and an additional sensor boolean indicates occupancy with high confidence.
- `occupancy.launch` is an executable launch file which launches the system’s ROS nodes
- `occupancy.launch` lines 7-8 may be uncommented to bag topic data while running the system for later analysis 


## Installation/Setup

### General Setup

Flash the Teensy with `seacube_teensy.ino` and, if used, the Arduino Uno with `seacube_arduino_uno.ino`. 

SeaCube’s RPi utilizes a Docker container running the Noetic distribution of the Robot Operating System (ROS) to host the two ROS nodes. Copy the contents of the repository’s `occupancy_sensor` ROS package to the `catkin_ws/src/` directory.

To remotely control the RPi while the docking station was deployed, a TP Link router was placed within the station’s housing. Connecting a laptop running PuTTY and the RPi to the router’s network enabled a Secure Shell (SSH) connection to manually run the launch file for testing. Alternatively, a Wi-Fi hotspot can be used for the network.

Ensure the proper USB connections are in place:
- If used, the Arduino Uno may be powered by any USB source, so long as it shares a ground with the RPi
- The Teensy must be connected to a RPi USB port for both power and serial communication. Take note of the port used (e.g. `/dev/ttyACM1`

### Repository Customizations

The repository has some code which is specific to SeaCube and can be removed/modified:
- `occupancy_pub_node.py` line 12 must be modified with the RPi USB port connected to the Teensy
- `occupancy_sub_node.py` does not use `imu_topic`. This data is intended for other SeaCube processes not related to occupancy detection. This topic and its associated variables may be removed from `occupancy_pub_node.py`. The XVec, YVec, and ZVec fields of `seacube_teensy.ino` may also be removed if desired, as this is their only purpose.
- `occupancy_sub_node.py` lines 28-36 can be deleted, as these define ROS Publishers/variables specific to SeaCube
- `occupancy_sub_node.py`’s `actuate()` method should be rewritten with the specific conditions that indicate AUV occupancy and the response to AUV occupancy
- `occupancy.launch` line 4 is SeaCube specific and may be removed

Additionally, it may be necessary to calibrate/modify the following threshold values:
- `occupancy_sub_node.py`’s `self.filter_threshold` variable must be adjusted depending on environmental factors (e.g. wave intensity) and the force at which the AUV collides with the station when docking
- `occupancy_sub_node.py`’s `self.charging_threshold` variable is the value that the averaged INA169 analog output must surpass to indicate alignment of the inductive coils. The analog value is proportional to the current circulating the inductive transmitting coil, which is dependent on the medium the coil is suspended in (see the system’s paper for more information).


## Usage

- Enter the Docker container running ROS Noetic over SSH
- `source setup.bash`
- Ensure the USB connections are in place as described in Installation/Setup.
- `sudo chmod 666 /dev/ttyACM1` (replace `ttyACM1` with the RPi USB port connected to the Teensy) to allow USB read/write permissions
- `export ROS_MASTER_URI=http://127.0.0.1:11311`
- `export ROS_HOSTNAME=127.0.0.1`
- Determine whether the data will be bagged by either commenting or uncommenting the rosbag line of the `occupancy.launch` file
- `roslaunch occupancy_sensor occupancy.launch` to run the launch file

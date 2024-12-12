#!/usr/bin/env python3

#Import Python modules
import rospy
import time
import serial
#Import ROS packages:
from std_msgs.msg import Bool, Int16, Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


class OccupancyPublisherNode:	
	def __init__(self, serial_obj):
		'''
		Initializes the occuapcny sensor system's publisher node
		Args:
			serial_obj: a serial object for reading sensor data from the 
						Teensy over USB seiral
		'''
		rospy.init_node("occupancy_publisher") #Initialize and name the publisher node
		
		#Class variables for USB serial reads
		self.line = ""
		self.last_line = "0,0,0,0,0,0,0,0,0" #Stores most recent serial data
		self.line_array = []
		self.ser = serial_obj
		
		#Create publishers:
		self.pub_switch = rospy.Publisher("switch_topic", Bool, queue_size=10)
		self.pub_rfid = rospy.Publisher("rfid_topic", Bool, queue_size=10)
		self.pub_charger = rospy.Publisher("charging_topic", Int16, queue_size=10)
		self.pub_imu = rospy.Publisher("imu_topic", Imu, queue_size=10)
		self.pub_filter = rospy.Publisher("filter_topic", Float32, queue_size=10)
		
		#Create messages:
		self.switch_msg = Bool()
		self.rfid_msg = Bool()
		self.charger_msg = Int16()
		self.imu_msg = Imu()
		self.filter_msg = Float32()
		
		self.rate = rospy.Rate(25) #Set publishing rate to 25 Hz
		print("Occupancy publisher node setup complete")
		
		
		
	def debug_print(self):
		'''
		Prints a formatted string containing the most recently collected sensor 
		data to the terminal
		'''
		data_string = str(self.switch_msg.data) + "," + str(self.rfid_msg.data) + "," + str(self.charger_msg.data)
		data_string = data_string + "," + str(self.imu_msg.linear_acceleration.x) + "," + str(self.imu_msg.linear_acceleration.y)
		data_string = data_string + "," + str(self.imu_msg.linear_acceleration.z) + "," + str(self.filter_msg.data)
		rospy.loginfo("Publishing: %s", data_string)	
		
			
				
	def publisher(self):
		'''
		Repeatedly reads USB serial input from the Teensy, parses the formatted 
		string, and publishes the data over five topics at a rate of 25 Hz. 
		If there is no new serial input from the Teensy, republishes the most
		recently collected data 
		'''
		while not rospy.is_shutdown():
			self.line = "" #Reset for current iteration
			
			if self.ser.in_waiting > 0: #New data available over serial
				#Read line from serial port, convert to string, strip whitespace:
				self.line = self.ser.readline().decode('utf-8').rstrip()
				self.last_line = self.line
			else: 	#No new data read over serial, will republish old data
				self.line = self.last_line
			
			#Parse self.line and fill message objects with data
			#Serial input format: [Switch],[RFID],[INA169],[XVec],[YVec],[ZVec],[HPFilter]
			self.line_array = self.line.split(",")
			if self.line_array[0] == "1":
				self.switch_msg.data = True
			else:
				self.switch_msg.data = False
			if self.line_array[1] == "1":
				self.rfid_msg.data = True
			else:
				self.rfid_msg.data = False
			self.charger_msg.data = int(self.line_array[2])
			
			self.imu_msg.header.stamp = rospy.Time.now() 	#Timestamp for IMU message header
			self.imu_msg.header.frame_id = "imu_link" 		#Default coordinate frame for IMU message header

			self.imu_msg.linear_acceleration = Vector3(float(self.line_array[3]), #x, y, z linear accelerations
													   float(self.line_array[4]), 
													   float(self.line_array[5])) 
			self.filter_msg.data = float(self.line_array[6])
			
			#Publish messages over topics:
			self.pub_switch.publish(self.switch_msg)
			self.pub_rfid.publish(self.rfid_msg)
			self.pub_charger.publish(self.charger_msg)
			self.pub_imu.publish(self.imu_msg)
			self.pub_filter.publish(self.filter_msg)	
			
			self.rate.sleep() #Pause here to maintain 25 Hz publish rate
		self.ser.close() #If loop breaks, close serial connection
			
			
			
if __name__ == '__main__':	
	#Initialize USB serial connection to Teensy:
	port = "/dev/ttyACM1"
	baud = 115200
	serial_obj = serial.Serial(port, baud, timeout=None) #Serial object
	
	#Teensy's .ino file will now restart
	time.sleep(3) #Wait for restart
	
	serial_obj.reset_input_buffer() #Clear serial buffer before reading
	
	pub = OccupancyPublisherNode(serial_obj)
	pub.publisher()
    
	

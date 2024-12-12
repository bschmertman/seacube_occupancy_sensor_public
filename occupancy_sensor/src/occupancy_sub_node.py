#!/usr/bin/env python3

#import Python modules:
import rospy
import time 
from gpiozero import LED
#Import ROS packages:
from std_msgs.msg import Bool, Int16, Float32
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer


class OccupancySubscriberNode:
	def __init__(self):
		'''
		Initializes the occupancy sensor system's subscriber node
		'''
		rospy.init_node("occupancy_subscriber") 	#Initialize and name subscriber node
		
		#Initialize topic to publish occupancy status for bagging
		self.pub_occupancy = rospy.Publisher("occupancy_topic", String, queue_size=10)
		self.occupancy_string = ""
		
		#Initalize topic publishing calculated INA169 analog-value averages
		self.pub_average_INA169 = rospy.Publisher("average_INA169_topic", Int16, queue_size=10)
		
		#Publisher objects to control raising/lowering of Seacube carriage:
		self.raise_carriage = rospy.Publisher("/seacube_actuator/timed_extend", Int16, queue_size=10)
		self.lower_carriage = rospy.Publisher("/seacube_actuator/timed_retract", Int16, queue_size=10)
		
		#Occupancy LED class variables:
		self.green_LED = LED(17)
		self.red_LED = LED(27)
		self.green_LED.off()	#Set default lighting
		self.red_LED.on()		#Set default lighting
		
		self.switch_occupancy = False		#Monitors if switch indicates occupancy
		self.rfid_occupancy = False			#Monitors if RFID indicates occupancy
		self.filter_occupancy = False		#Monitors if IMU high pass filter indicates occupancy
		self.charging_occupancy = False		#Monitors if inductive module indicates occupancy
		self.occupancy_status = False		#Uses above variables to determine if LoCO has docked
		self.carriage_raised = False		#Stores current status of carriage
		self.carriage_timer = 0				#Timer used to raise/lower the carriage
		
		#Switch class variables
		self.switch_start_press = 0		#Timer variable measuring if switch depressed for >=2 seconds
		
		#IMU class variables
		self.filter_timer = 0			#Timer to indicate occupancy for 5 seconds after collision detected		
		self.filter_threshold = 1		#Threshold that filter output must surpass to indicate a collison
		self.first_iteration = 1		#indicates the loop is iterating for the very first time
		
		#INA169/inductive module class variables 
		self.charging_avg = 0			#Stores average of 50 most recent analog readings
		self.charging_sum = 0			#Variable for calculating the average
		self.charging_counter = 0		#Variable for calculating the average
		self.charging_threshold = 45	#Threshold that self.charging_avg must surpass to indicate coil alignment
	
		#Subscribers to topics from publsiher node:
		self.sub_switch = Subscriber("switch_topic", Bool)
		self.sub_rfid = Subscriber("rfid_topic", Bool)
		self.sub_charging = Subscriber("charging_topic", Int16)
		self.sub_filter = Subscriber("filter_topic", Float32)
		
		#Class variables storing most recent data from publisher node
		self.switch = False
		self.rfid = False
		self.charging = 0
		self.filter = 0.0
			
		#Attach all subscribers to a single callback function
		self.sub_list = [self.sub_switch, self.sub_rfid, self.sub_charging, self.sub_filter]
		self.ats = ApproximateTimeSynchronizer(self.sub_list, queue_size=10, slop=0.03, allow_headerless=True)
		self.ats.registerCallback(self.common_callback)
		
		print("Occupancy subscriber node setup complete")
	
	
	
	def debug_print(self):
		'''
		Prints a formatted output of the most recently received sensor data 
		from the publisher node to the terminal
		'''
		rospy.loginfo("Switch: %s", self.switch.data)
		rospy.loginfo("RFID: %s", self.rfid.data)
		rospy.loginfo("Charging: %s", self.charging.data)
		rospy.loginfo("Filter: %s", self.filter.data)
		rospy.loginfo("\n")
		
	
	
	
	def occupancy(self): 		
		"""
		Processes the received sensor data from the publisher node to determine
		which sensors indicate the occupancy of LoCO. Updates class variables 
		accordingly.
		"""
		#------------------------Compute INA169/charger occupancy status----------------------
		
		#Compute average of 50 analog outputs from INA169
		if self.charging_counter >= 50: 	#Ready to update average analog value	
			self.charging_avg = self.charging_sum / self.charging_counter
			self.pub_average_INA169.publish(int(self.charging_avg))	#Publish calculated average for bagging
			self.charging_sum = 0 			#Reset for next average calculation
			self.charging_counter = 0
		else: 								#Continue averaging charging value
			self.charging_sum += self.charging.data
			self.charging_counter += 1
			
		if self.charging_avg >= self.charging_threshold:
			self.charging_occupancy = True
		else:
			self.charging_occupancy = False
	
		#-------------------------Compute IMU HP Filter occupancy status---------------------- 
		
		if(self.first_iteration):	#First time iterating, do not compare to threshold
			self.first_iteration = 0
		else:						
			if (not self.filter_occupancy) and (self.filter.data >= self.filter_threshold):	#New collision detected
				self.filter_occupancy = True
				self.filter_timer = time.time() + 5								#5 seconds from now
			elif self.filter_occupancy and time.time() >= self.filter_timer:	#5 seconds have passed, reset
				self.filter_occupancy = False
			else:
				pass
		
		#----------------------------Compute switch occupancy status------------------------- 
		
		if not self.switch.data:									#Switch not being depressed
			self.switch_start_press = 0
			self.switch_occupancy = False
		else:														#Switch is depressed
			if self.switch_start_press == 0:						#Switch depressed for first time in a while				
				self.switch_start_press = time.time()				
				self.switch_occupancy = False
			else:													#Switch has been held down for some time
				if (time.time() - self.switch_start_press) >= 2: 	#If switch depressed for at least 2 seconds
					self.switch_occupancy = True
				else:												#Switch has not been depressed for at least 2 seconds
					self.switch_occupancy = False				

		#----------------------------Compute RFID occupancy status------------------------- 
		
		if self.rfid.data:
			self.rfid_occupancy = True
		else:
			self.rfid_occupancy = False
			
		#--------------------------Print triggered sensors to terminal----------------------- 
			
		self.occupancy_string = "Triggered sensors: "
		
		if self.switch_occupancy:
			self.occupancy_string += " Switch "
		if self.rfid_occupancy:
			self.occupancy_string += " RFID "
		if self.charging_occupancy:
			self.occupancy_string += " INA169 "
		if self.filter_occupancy:
			self.occupancy_string += " Filter "
		
				
		
	def actuate(self):
		"""
		Uses processing performed by self.occupancy() to determine whether LoCO
		is correctly positioned within the carriage. If so, this method changes the 
		occupancy LED indicators accordingly, raises the carriage, and lowers it again
		"""
		#--------------------------Determine if LoCO is in carriage--------------------------
		
		#For pool trial, carriage is only raised if below condition is met
		if self.switch_occupancy and self.rfid_occupancy:
			self.occupancy_status = True
		else:
			self.occupancy_status = False
		
		#--------------------------Actuate carriage and LED indicators-------------------------
		if self.occupancy_status and (not self.carriage_raised) and (time.time() > self.carriage_timer):
			self.green_LED.on()
			self.red_LED.off()
			self.occupancy_string += " (Raising carriage) "
			self.raise_carriage.publish(40) 		#Publish message to raise carriage
			self.carriage_raised = True
			self.carriage_timer = time.time() + 45 	#Do not actuate carriage again until current action completes
		if(self.carriage_raised and (time.time() >= self.carriage_timer)):
		    self.green_LED.off()
		    self.red_LED.on()
		    self.occupancy_string += " (Lowering carriage) "
		    self.lower_carriage.publish(40) 		#Publish message to lower carriage
		    self.carriage_raised = False
		    self.carriage_timer = time.time() + 60 	#Provide additional time for LoCO to exit carriage
		
		
		
	def common_callback(self, ats_switch, ats_rfid, ats_charging, ats_filter):
		"""
		Common callback function which is called for every set of messages received 
		from the publisher node
		Args:
			ats_switch: synchronized switch message received from publisher
			ats_rfid: synchronized RFID message received from publisher
			ats_charging: synchronized INA169 output message received from publisher
			ats_filter: synchronized IMU HP filter message received from publisher
		"""
		#Update class variables with received syncrhonized data
		self.switch = ats_switch
		self.rfid = ats_rfid
		self.charging = ats_charging
		self.filter = ats_filter
	
		self.occupancy() 	#Process raw occupancy sensor data
		self.actuate()		#Actuate carriage and LED indicators accordingly
		
		self.pub_occupancy.publish(self.occupancy_string) 	#Publish triggered sensors for bagging
		rospy.loginfo(self.occupancy_string)				#Print triggered sensors to terminal
		
		
		
		
if __name__ == '__main__':
	sub = OccupancySubscriberNode()
	rospy.spin() #Keeps node running until shutdown
	
	


#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
# inspired from https://github.com/qboticslabs/Chefbot_ROS_pkg/blob/master/chefbot/chefbot_bringup/scripts/launchpad_node.py


#Python client library for ROS
import rospy
import sys
import time
import math

#This module helps to receive values from serial port
from SerialDataGateway import SerialDataGateway
#Importing ROS data types
from std_msgs.msg import Int16,Int32, Int64, Float32, String, Header, UInt64
#Importing ROS data type for IMU
from sensor_msgs.msg import Imu

#Class to handle serial data from Arduino and converted to ROS topics
class ArduinoClient(object):
	
	def __init__(self):
		#print "Initializing ArduinoClient Class"

		#Sensor variables
		self._Counter = 0

		self._left_encoder_value = 0
		self._right_encoder_value = 0

		#self._battery_value = 0  # TODO: for later 

		self._left_wheel_speed_ = 0
		self._right_wheel_speed_ = 0

		#self._LastUpdate_Microsec = 0
		#self._Second_Since_Last_Update = 0

		self.robot_heading = 0

		#Get serial port and baud rate of Arduino
		port = rospy.get_param("~port", "/dev/ttyACM0")
		baudRate = int(rospy.get_param("~baudRate", 115200))

		rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))
		#Initializing SerialDataGateway with port, baudrate and callback function to handle serial data
		self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)
		rospy.loginfo("Started serial communication")
		

        #Subscribers and Publishers

		#Publisher for left and right wheel encoder values
		self._Left_Encoder = rospy.Publisher('lwheel',Int64,queue_size = 10)		
		self._Right_Encoder = rospy.Publisher('rwheel',Int64,queue_size = 10)		

		#Publisher for Battery level(for upgrade purpose)
		#self._Battery_Level = rospy.Publisher('battery_level',Float32,queue_size = 10)
	
		#Publisher for entire serial data
		self._SerialPublisher = rospy.Publisher('serial', String,queue_size=10)
		
        #Subscribers and Publishers of IMU data topic

		self.frame_id = '/base_link'

	        self.cal_offset = 0.0
        	self.orientation = 0.0
        	self.cal_buffer =[]
        	self.cal_buffer_length = 1000

		self.deltat = 0
		self.lastUpdate = 0

        #Speed subscriber
		self._left_motor_speed = rospy.Subscriber('left_wheel_speed',Float32,self._Update_Left_Speed)
		self._right_motor_speed = rospy.Subscriber('right_wheel_speed',Float32,self._Update_Right_Speed)


	def _Update_Left_Speed(self, left_speed):
		self._left_wheel_speed_ = left_speed.data
		rospy.loginfo(left_speed.data)
		speed_message = 's,%d,%d\n' %(int(self._left_wheel_speed_),int(self._right_wheel_speed_))
		self._WriteSerial(speed_message)

				

	def _Update_Right_Speed(self, right_speed):
		self._right_wheel_speed_ = right_speed.data
		rospy.loginfo(right_speed.data)
		speed_message = 's,%d,%d\n' %(int(self._left_wheel_speed_),int(self._right_wheel_speed_))
		self._WriteSerial(speed_message)

	def Reset_Speed(self):
		rospy.loginfo("reset speed")
		self._WriteSerial("s,0,0\n")

    #Calculate orientation from accelerometer and gyrometer
	def _HandleReceivedLine(self,  line):
		self._Counter = self._Counter + 1
		self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))


		if(len(line) > 0):

			lineParts = line.split('\t')
			try:
				if(lineParts[0] == 'e'):
					self._left_encoder_value = long(lineParts[1])
					self._right_encoder_value = long(lineParts[2])

					self._Left_Encoder.publish(self._left_encoder_value)
					self._Right_Encoder.publish(self._right_encoder_value)

#				if(lineParts[0] == 'b'):
#					self._battery_value = float(lineParts[1])
#					self._Battery_Level.publish(self._battery_value)

				
			except:
				rospy.logwarn("Error in Sensor values")
				rospy.logwarn(lineParts)
				pass
			

	def _WriteSerial(self, message):
		self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
		self._SerialDataGateway.Write(message)


	def Start(self):
		rospy.logdebug("Starting")
		self._SerialDataGateway.Start()


	def Stop(self):
		rospy.logdebug("Stopping")
		self._SerialDataGateway.Stop()
		


	
#	def Subscribe_Speed(self):
#		a = 1
##		print "Subscribe speed"


#	def Reset_Arduino(self):
#		print "Reset"
#		reset = 'r\r'
#		self._WriteSerial(reset)
#		time.sleep(1)
#		self._WriteSerial(reset)
#		time.sleep(2)



#	def Send_Speed(self):
##		print "Set speed"
#		a = 3


if __name__ =='__main__':
	rospy.init_node('arduino_ros',anonymous=True)
	arduino = ArduinoClient()
	try:
		arduino.Start()	
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("Error in main function")


	#arduino.Reset_Arduino()
	arduino.Reset_Speed()
	arduino.Stop()



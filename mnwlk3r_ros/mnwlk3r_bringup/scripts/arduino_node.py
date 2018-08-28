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

# for messages
#from std_msgs.msg import Int16,Int32, Int64, Float32, String, Header, UInt64
from std_msgs.msg import String
from mnwlk3r_bringup.msg import RPMMotors, EncoderCounts 


#Class to handle serial data from Arduino and converted to ROS topics
class ArduinoClient(object):
    
    def __init__(self):
        #print "Initializing ArduinoClient Class"


        #Get serial port and baud rate of Arduino
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baudRate = int(rospy.get_param("~baudRate", 115200))

        rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))
        #Initializing SerialDataGateway with port, baudrate and callback function to handle serial data
        self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)
        rospy.loginfo("Started serial communication")
        

        #Subscribers and Publishers

        #Publisher for left and right wheel encoder values
        self._Encoder_Counts_Pulisher = rospy.Publisher('encoder_counts',EncoderCounts,queue_size = 10)        

        #Publisher for entire serial data
        self._SerialPublisher = rospy.Publisher('serial', String,queue_size=10)
        
        #Speed subscriber
        self._rpm_motor_speed = rospy.Subscriber('wheels_rpm_target',RPMMotors,self._update_speed_rpm_target)
         


    def _update_speed_rpm_target(self, rpm_message):
        left_rpm  = rpm_message.left
        right_rpm = rpm_message.right
        # >>> 's,%02.2f,%f\n' %(float(12.1301),float(125.0))
        speed_message = 's,%02.2f,%02.2f\n' %(float(left_rpm),float(right_rpm))
        self._WriteSerial(speed_message)


    def Reset_Speed(self):
        rospy.loginfo("reset speed")
        self._WriteSerial("s,0,0\n")

    #received data from arduino
    def _HandleReceivedLine(self,  line):

        
        #Serial.print(Setpoint_right); //double
        #Serial.print(Input_right); //double
        #Serial.print(Setpoint_left); //double
        #Serial.print(Input_left); // double
        #Serial.print(Left_Encoder_Ticks); // long
        #Serial.print(Right_Encoder_Ticks); //long
        #Serial.print(now); // long

        if(len(line) > 0):

            #lineParts = line.split('\t')
            lineParts = line.split(',')
            try:
                left_encoder_value = long(lineParts[4])
                right_encoder_value = long(lineParts[5])
                rospy.loginfo(str(left_encoder_value)+","+str(right_encoder_value))
                
                msg = EncoderCounts()
                msg.left = left_encoder_value
                msg.right = right_encoder_value
                self._Encoder_Counts_Pulisher.publish(msg)

                
            except:
                rospy.logwarn("Error in Arduino values")
                rospy.logwarn(lineParts)
                pass
            

    def _WriteSerial(self, message):
        self._SerialDataGateway.Write(message)


    def Start(self):
        rospy.logdebug("Starting")
        self._SerialDataGateway.Start()


    def Stop(self):
        rospy.logdebug("Stopping")
        self._SerialDataGateway.Stop()
        


    
#    def Subscribe_Speed(self):
#        a = 1
##        print "Subscribe speed"


#    def Reset_Arduino(self):
#        print "Reset"
#        reset = 'r\r'
#        self._WriteSerial(reset)
#        time.sleep(1)
#        self._WriteSerial(reset)
#        time.sleep(2)



#    def Send_Speed(self):
##        print "Set speed"
#        a = 3


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



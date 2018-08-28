#!/usr/bin/env python

# inspiraton from chefbot => https://github.com/qboticslabs/Chefbot_ROS_pkg

import rospy
import roslib
#from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist 

#from exercise_23.msg import Age #Import Age message from the exercise_23 package
from mnwlk3r_bringup.msg import RPMMotors #Import Age message from the exercise_23 package

import math  # for math.pi

class TwistToMotors():

    def __init__(self):
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.dx = 0.0
        self.dr = 0.0
        
        self.timeout_cmd = 0.25 # how many seconds we tolerate without Twist message before we send a wheel_rpm zero (stop the robot
        self.last_command_time = 0.0
        
        #self.w = rospy.get_param("~base_width", 0.328)
        self.w = rospy.get_param("~base_width", 0.328)
        wheel_diameter = rospy.get_param("wheel_diameter", 0.136)
        
        # now for rpm
        # 1 rpm <=> 1 rev per 60 sec = (diameter * math.pi) / 60 =  (0.136 * 3.14) / 60 = 0.0071 m/s
        self.ms_to_rpm = (wheel_diameter * math.pi)/60.0
    
        self.pub_motor = rospy.Publisher('wheels_rpm_target', RPMMotors,queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
    
    
        self.rate = rospy.get_param("~rate", 20)
        self.left = 0
        self.right = 0

        
    def spin(self):
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
    
        ###### main loop  ######
        while not rospy.is_shutdown():
        
            while not rospy.is_shutdown(): #and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()


            
    def spinOnce(self):
        # reset speed values if we haven't received any for a while
        if (rospy.Time.now().to_sec() - self.last_command_time > self.timeout_cmd):
            self.dx = self.dx * 0.6
            self.dr = self.dx * 0.6         
        
        #if (rospy.Time.now().to_sec() - self.last_command_time > self.timeout_cmd*3):
        #    self.dx = 0
        #    self.dr = 0         

        # dx = (l + r) / 2
        # dr = (r - l) / w
            
        self.right = 1.0 * self.dx + self.dr * self.w / 2 
        self.left = 1.0 * self.dx - self.dr * self.w / 2
        # above is speed in m/s ?
        
        self.right = self.right / self.ms_to_rpm
        self.left  = self.left  / self.ms_to_rpm
        
        #rospy.loginfo("publishing: (%d, %d)", self.left, self.right) 
        msg = RPMMotors()
        msg.left = self.left
        msg.right = self.right
        self.pub_motor.publish(msg)
            

    def twistCallback(self,msg):
        #rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.last_command_time = rospy.Time.now().to_sec()
    
if __name__ == '__main__':
    """ main """
    twistToMotors = TwistToMotors()
    twistToMotors.spin()

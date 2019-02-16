#!/usr/bin/env python
import rospy
import time
import wiringpi
from std_msgs.msg import String
import math
from geometry_msgs.msg import Twist

def callback(msg):
	# msg.linear.x  # -0.2 0.2 ish m/s
	# msg.angular.z  # rad/s  -1 1/ 

	wiringpi.pwmWrite(18, math.floor(150 + msg.linear.x * 10))

	rospy.loginfo("desired cmd_vel x is  {}  ang z is  {}".format(msg.linear.x,msg.angular.z))
    
def setup():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
	rospy.init_node('motor_control', anonymous=True)

	rospy.Subscriber("cmd_vel", Twist, callback)
	# use 'GPIO naming'
	wiringpi.wiringPiSetupGpio()
 
	# set #18 to be a PWM output
	wiringpi.pinMode(18, wiringpi.GPIO.PWM_OUTPUT)
 
	# set the PWM mode to milliseconds stype
	wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
 
	# divide down clock
	wiringpi.pwmSetClock(192)
	wiringpi.pwmSetRange(2000)
 
	delay_period = 0.01  

if __name__ == '__main__':
	setup()	
	rospy.spin()

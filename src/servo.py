#!/usr/bin/env python
import rospy
import time
import math
from RPIO import PWM
from geometry_msgs.msg import Twist

class MotorControl(Object):

	def __init__(self):
		rospy.init_node('motor_control', anonymous=True)
		self.sub=rospy.Subscriber("cmd_vel", Twist, self.callback) 				
		self.servo = PWM.Servo()		

	def callback(self,msg):
		# msg.linear.x  # -0.2 0.2 ish m/s
		# msg.angular.z  # rad/s  -1 1/ 
		# assuming left wheel on pin 18 , and right wheel on in 19
		
		ms_value_l = math.floor(1500 + (msg.linear.x * 100 - msg.angualr.z*100/2) )
		ms_value_r = math.floor(1500 + (msg.linear.x * 100 - msg.angualr.z*100/2) )

		ms_value_l=min(2000,max(1000,ms_value_l))
		ms_value_r=min(2000,max(1000,ms_value_r))		
		
		self.servo.set_servo(17, ms_value_l)
		self.servo.set_servo(18, ms_value_r)
		
		# self.servo.stop_servo(18)
		# self.servo.stop_servo(19)

		rospy.loginfo("desired cmd_vel x is  {}  ang z is  {}".format(msg.linear.x,msg.angular.z))		

if __name__ == '__main__':
	PWM.setup()
	mt_ctrl=MotorControl()
	rospy.spin()
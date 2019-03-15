#!/usr/bin/env python
import rospy
import time
import math
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist

class MotorControl(object):

	class Servo(object):
		def __init__(self,pin,freq=50):		
			GPIO.setup(pin, GPIO.OUT)
			self.freq=freq
			self.pwm=GPIO.PWM(pin, freq)
			self.pwm.start(0)

		def __del__(self):
			self.pwm.stop(0)	
		
		def set_ms(self, ms):
			ms=min(max(1000),2000)
			period=(int)(1000/self.freq)			
			self.pwm.ChangeDutyCycle(ms/period)	

	def __init__(self):
		rospy.init_node('motor_control', anonymous=True)
		self.sub=rospy.Subscriber("cmd_vel", Twist, self.callback)
		GPIO.setmode(GPIO.BOARD)
		self.left_servo=Servo(17)	
		self.right_servo=Servo(18)					

	def __del__(self):	
		GPIO.cleanup()

	def callback(self,msg):
		# msg.linear.x  # -0.2 0.2 ish m/s
		# msg.angular.z  # rad/s  -1 1/ 
		# assuming left wheel on pin 18 , and right wheel on in 19
		
		ms_value_l = math.floor(1500 + (msg.linear.x * 100 - msg.angualr.z*100/2) )
		ms_value_r = math.floor(1500 + (msg.linear.x * 100 - msg.angualr.z*100/2) )

		ms_value_l=min(2000,max(1000,ms_value_l))
		ms_value_r=min(2000,max(1000,ms_value_r))		
		
		self.left_servo.set_servo(ms_value_l)
		self.right_servo.set_servo(ms_value_r)		
		rospy.loginfo("desired cmd_vel x is  {}  ang z is  {}".format(msg.linear.x,msg.angular.z))		

if __name__ == '__main__':	
	mt_ctrl=MotorControl()
	rospy.spin()
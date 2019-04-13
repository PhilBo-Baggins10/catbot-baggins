#!/usr/bin/env python
import rospy
import time
import math
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist

class MotorControl(object):

	class Servo(object):
		def __init__(self,pin,freq=100, name):		
			GPIO.setup(pin, GPIO.OUT)
			self.name=name
			self.freq=freq
			self.pwm=GPIO.PWM(pin, freq)
			self.pwm.start(0)

		def __del__(self):
			self.pwm.stop(0)	
		
		def set_ms(self, ms):
			ms=min(max(1.,ms),2.)
			ms= (not (ms<1.55 and ms>1.45))*ms
			
			period=(float)(1000/self.freq)
			rospy.loginfo("{} motor is {} % DC and ms is {}".format( self.name,(float)(ms/period)*100.0,ms)	)		
			self.pwm.ChangeDutyCycle((float)(ms/period)*100.0)	

	def __init__(self):
		rospy.init_node('motor_control', anonymous=True)
		self.sub=rospy.Subscriber("cmd_vel", Twist, self.callback)
		GPIO.setmode(GPIO.BCM)
		self.left_servo=MotorControl.Servo(18,"left")	
		self.right_servo=MotorControl.Servo(19,"right")					

	def __del__(self):	
		GPIO.cleanup()

	def callback(self, msg):
		# msg.linear.x  # -0.2 0.2 ish m/s
		# msg.angular.z  # rad/s  -1 1/ 
		# assuming left wheel on pin 18 , and right wheel on in 19
		
		ms_value_l = math.floor(1500 + (msg.linear.x * 100 - msg.angular.z*100/2) )
		ms_value_r = math.floor(1500 - (msg.linear.x * 100 + msg.angular.z*100/2) )

		ms_value_l= min(2000.,max(1000.0,ms_value_l))
		ms_value_r= min(2000.,max(1000.,ms_value_r))		

		
		self.left_servo.set_ms(ms_value_l/1000.0)
		self.right_servo.set_ms(ms_value_r/1000.0)		
		rospy.loginfo("desired cmd_vel x is  {}  ang z is  {}".format(msg.linear.x,msg.angular.z))		

if __name__ == '__main__':	
	mt_ctrl=MotorControl()
	rospy.spin()
	GPIO.cleanup()
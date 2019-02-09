#!/usr/bin/env python
import rospy
import time
import wiringpi
from std_msgs.msg import String

def callback(data):
	wiringpi.pwmWrite(18, 100)
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("chatter", String, callback)
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
   	 # spin() simply keeps python from exiting until this node is stopped
    	rospy.spin()

if __name__ == '__main__':
    listener()



#!/usr/bin/env python
import time
import wiringpi

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

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
	wiringpi.pwmWrite(18, 150)

   	 # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()



#!/usr/bin/env python

PKG = 'aribo'

import roslib; roslib.load_manifest(PKG)

from serial import Serial
import time
from threading import Thread

import rospy
from std_msgs.msg import Float32

class ReadWaspMote():

    def __init__(self):
        self.is_running = True
	self.ser = Serial("/dev/ttyUSB0", 115200)
        rospy.init_node('read_mote', anonymous=True)
        self.mote_temperature = rospy.Publisher('/temperature', Float32, queue_size=10)
	self.rate = rospy.Rate(10) # 10hz
	self.temperature = 0

    def update_mote_values(self):

	while self.is_running == True and not rospy.is_shutdown():
	    line = self.ser.readline()
	    if "TEMP" in line:
	    	self.header, self.temperature = line.split(",")
	    	print(float(self.temperature))
	    	self.mote_temperature.publish(float(self.temperature))
	    self.rate.sleep()

if __name__ == '__main__':
    try:
    	read_mote = ReadWaspMote()
    	t = Thread(target=read_mote.update_mote_values)
	t.start()
	rospy.spin()
    	read_mote.alive = False
	t.join()
    except rospy.ROSInterruptException: pass

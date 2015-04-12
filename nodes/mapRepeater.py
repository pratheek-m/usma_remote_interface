#!/usr/bin/env python

PACKAGE = 'aribo'
import roslib
roslib.load_manifest(PACKAGE)
import rospy

from nav_msgs.msg import OccupancyGrid
from threading import Thread
import time

# actually controls the velocity based on the multiple inputs provided
class map_frequency_shift:
	def __init__(self):
		rospy.init_node('2d_map_repeater', anonymous=True)
		self.out_map_pub = rospy.Publisher('/fast_map', OccupancyGrid)
		self.map_buffer = None
		rospy.Subscriber('/slow_map', OccupancyGrid, self.mapCb)
		self.alive=True

	def mapCb(self, data):
		self.map_buffer = data

	def updateMap(self):
		rate = rospy.Rate(1.0)
		while self.alive == True and not rospy.is_shutdown():
		   if not self.map_buffer == None:
			self.out_map_pub.publish(self.map_buffer)
			rate.sleep()
			
if __name__ == '__main__':
	try:
		mapUpdater = map_frequency_shift()
		t = Thread(target=mapUpdater.updateMap)
		print "thread start"
		t.start()
		rospy.spin()
		mapUpdater.alive = False
		t.join()
	except rospy.ROSInterruptException: pass

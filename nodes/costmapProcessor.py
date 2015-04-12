#!/usr/bin/env python

PACKAGE = 'aribo'
import roslib
roslib.load_manifest(PACKAGE)
import rospy

from nav_msgs.msg import OccupancyGrid
from math import sin, cos, pi, sqrt, asin, atan2, log
import time
import numpy as np
from scipy.ndimage.morphology import grey_dilation
from scipy.ndimage import gaussian_filter

# actually controls the velocity based on the multiple inputs provided
class costmap_processing:
	def __init__(self):
		rospy.init_node('costmap_processing', anonymous=True)

		self.costmapPub = rospy.Publisher('/graded_map',OccupancyGrid)
		rospy.Subscriber('/binary_map', OccupancyGrid, self.costmapCb)

	def costmapCb(self, data):
		# convert to a convinient numpy array of the right shape
		np_original = np.array(data.data).reshape(data.info.height, data.info.width)
		np_costmap = np.array(data.data).reshape(data.info.height, data.info.width)
		fp = np.ones((7,7))
		fp = fp/2.0
		fp[1][1] = 1.0
		np_costmap = grey_dilation(np_costmap,footprint=fp)
		np_costmap = gaussian_filter(np_costmap,3.0)	
		np_costmap = grey_dilation(np_costmap,footprint=fp)

	
		# setup the publishable costmap
		new_costmap = OccupancyGrid()
		new_costmap.info = data.info
		new_costmap.header = data.header
		new_costmap.data = np_costmap.reshape(1,data.info.width*data.info.height)
		new_costmap.data = new_costmap.data.astype(int).tolist()
		new_costmap.data = new_costmap.data[0]
		new_costmap.data[ new_costmap.data > 100 ] = 100
		self.costmapPub.publish(new_costmap)

if __name__ == '__main__':
	try:
		move = costmap_processing()
		rospy.spin()
	except rospy.ROSInterruptException: pass

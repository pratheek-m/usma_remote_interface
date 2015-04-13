#!/usr/bin/env python

PACKAGE = 'aribo'
import roslib
roslib.load_manifest(PACKAGE)
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from math import sin, cos, pi, sqrt, asin, atan2, log
from threading import Thread
import time
from Queue import Queue
import tf

# actually controls the velocity based on the multiple inputs provided
class nav:
	def __init__(self):
		rospy.init_node('nav_stack', anonymous=True)
		self.robotCommandPub = rospy.Publisher('/VC/2DNav',Twist)

		self.nextGoal = None
		self.goalQueue = Queue()
		rospy.Subscriber('/path', Path, self.pathCb)
		self.curPose = None
		# just acts as a trigger
		rospy.Subscriber('/tf', TFMessage, self.poseCb)
		self.alive=True

	def pathCb(self, data):
		print data
		if self.nextGoal == None:
			for pose in data.poses:
				self.goalQueue.put(pose)
	
	def poseCb(self, data):
		for tf_ in data.transforms:
			if tf_.header.frame_id == "/map":
				self.curPose = tf_.transform

	def navigation(self):
		rate = rospy.Rate(100)
		item = self.goalQueue.get()
		while self.alive == True and not rospy.is_shutdown():
			if abs(self.curPose.translation.x-item.position.x)<0.1 and abs(self.curPose.translation.y-item.position.y)<0.1:
				item = self.goalQueue.get()
			cmd = Twist()
			if poseAngle(item,self.curPose)>0.1:
				cmd.angular.z=-0.1
			
			self.robotCommandPub.publish(cmd)
				
			# pull next goal off of queue if it's ready
			rate.sleep()
			

def poseDist(p1, p2):
	dx = p2.position.x-p1.position.x
	dy = p2.position.y-p1.position.y
	return sqrt((dx*dx)+(dy*dy))
def poseAngle(p1, p2):
	dx = p2.position.x-p1.translation.x
	dy = p2.position.y-p1.translation.y
	return atan2(dy,dx)

if __name__ == '__main__':
	try:
		move = nav()
		t = Thread(target=move.navigation)
		t.start()
		rospy.spin()
		move.alive = False
		t.join()
	except rospy.ROSInterruptException: pass

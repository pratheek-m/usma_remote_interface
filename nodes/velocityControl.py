#!/usr/bin/env python

PACKAGE = 'aribo'
import roslib
roslib.load_manifest(PACKAGE)
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import sin, cos, pi, sqrt, asin, atan2, log
from threading import Thread
import time

# actually controls the velocity based on the multiple inputs provided
class velocityControl:
	def __init__(self):
		self.update_freq = rospy.get_param('pioneer/updateFreq')
		self.teleopGain = rospy.get_param('pioneer/teleopGain')
		self.obstacleGain = rospy.get_param('pioneer/obstacleGain')
		self.out_twist_pub = rospy.Publisher('/VC/out', Twist)
		self.outTwist = Twist()
		self.outTwist.linear.x = 0.0
		self.outTwist.linear.y = 0.0
		self.outTwist.linear.z = 0.0
		self.outTwist.angular.x = 0.0 # roll
		self.outTwist.angular.y = 0.0 # pitch
		self.outTwist.angular.z = 0.0 # yaw
		rospy.init_node('velocity_control', anonymous=True)
		self.teleopData = None
		self.teleopTime = None
		rospy.Subscriber('/VC/teleop', Twist, self.teleop)
		self.obstacleData = None
		rospy.Subscriber('/VC/obstacle', Twist, self.obstacle)
		self.state = "teleop"
		rospy.Subscriber('VC/state', String, self.state) 
		self.alive=True

	def teleop(self, data):
		self.teleopTime = time.time()
		self.teleopData = data

	def state(self, data):
		self.state = data.data

	def obstacle(self,data):
		self.obstacleData = data

	def updateCommandVelocity(self):
		rate = rospy.Rate(self.update_freq)
		while self.alive == True and not rospy.is_shutdown():
		   self.outTwist.linear.x = 0.0
		   self.outTwist.angular.z = 0.0
		   if self.obstacleData and self.teleopData:
			self.outTwist.linear.x += self.obstacleData.linear.x*self.obstacleGain
			self.outTwist.angular.z += self.obstacleData.angular.z*self.obstacleGain
			if self.state=="teleop": # and abs(self.obstacleData.linear.x)<2.0 and abs(self.obstacleData.angular.z)<2.0:
			    if (time.time()-self.teleopTime)<1.0:
				self.outTwist.linear.x += (self.teleopData.linear.x*self.teleopGain)
				self.outTwist.angular.z += (self.teleopData.angular.z*self.teleopGain)
			self.outTwist.linear.x = -self.outTwist.linear.x
			self.outTwist.angular.z = -self.outTwist.angular.z
			self.out_twist_pub.publish(self.outTwist)
			rate.sleep()
			
		time.sleep(self.sleep_time)
if __name__ == '__main__':
	try:
		move = velocityControl()
		t = Thread(target=move.updateCommandVelocity)
		t.start()
		rospy.spin()
		move.alive = False
		t.join()
	except rospy.ROSInterruptException: pass

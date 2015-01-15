#!/usr/bin/env python

PACKAGE = 'chris_pioneer_control'
import roslib
roslib.load_manifest(PACKAGE)
import rospy

from geometry_msgs.msg import Twist
from math import sin, cos, pi, sqrt, asin, atan2, log
from threading import Thread

# actually controls the velocity based on the multiple inputs provided
class velocityControl:
	def __init__(self):
		self.sleep_time = 1 / rospy.get_param('pioneer/updateFreq')
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
		rospy.Subscriber('/VC/teleop', Twist, self.teleop)
		self.obstacleData = None
		rospy.Subscriber('/VC/obstacle', Twist, self.obstacle)
		self.alive=True

	def teleop(self, data):
		self.teleopData = data

	def obstacle(self,data):
		self.obstacleData = data

	def updateCommandVelocity(self):
		while self.alive == True and not rospy.is_shutdown():
			self.outTwist.linear.x = 0.0
			self.outTwist.angular.z = 0.0
			if self.teleopData:
				self.outTwist.linear.x += self.teleopData.linear.x*self.teleopGain
				self.outTwist.angular.z += self.teleopData.angular.z*self.teleopGain
			if self.obstacleData:
				self.outTwist.linear.x += self.obstacleData.linear.x*self.obstacleGain
				self.outTwist.angular.z += self.obstacleData.angular.z*self.obstacleGain
			self.out_twist_pub.publish(self.outTwist)
			
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

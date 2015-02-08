#!/usr/bin/env python

PACKAGE = 'aribo'
import roslib
roslib.load_manifest(PACKAGE)
import rospy

from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String
from math import sin, cos, pi, sqrt, asin, atan2, log
from threading import Thread
import time
from Queue import Queue

# actually controls the velocity based on the multiple inputs provided
class nav:
	def __init__(self):
		rospy.init_node('nav_stack', anonymous=True)
		self.update_freq = rospy.get_param('pioneer/updateFreq')

		self.robotCommandPub = rospy.Publisher('/VC/2DNav',Twist)
		self.outTwist = Twist()
                self.outTwist.linear.x = 0.0
                self.outTwist.linear.y = 0.0
                self.outTwist.linear.z = 0.0
                self.outTwist.angular.x = 0.0 # roll
                self.outTwist.angular.y = 0.0 # pitc		

		self.webFeedback_pub = rospy.Publisher('/aribo/feedback', MoveBaseActionFeedback)
		self.webFeedback = MoveBaseActionFeedback()
		self.webFeedback.header.frame_id = 'map'
		self.webFeedback.feedback.base_position.pose.position.x = 0.0
		self.webFeedback.feedback.base_position.pose.position.y = 0.0
		self.webResult_pub = rospy.Publisher('/aribo/result', MoveBaseActionResult)
		self.webResult = MoveBaseActionResult()


		self.nextGoal = None
		self.goalQueue = Queue()
		rospy.Subscriber('/aribo/goal', MoveBaseActionGoal, self.goalCb)
		self.curPose = None
		rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, self.poseCb)
		self.alive=True

	def goalCb(self, data):
		self.goalQueue.put(data)

	def poseCb(self, data):
		self.webFeedback.feedback.base_position.pose = data.pose.pose

	def navigation(self):
		rate = rospy.Rate(self.update_freq)
		item = self.goalQueue.get()
		while self.alive == True and not rospy.is_shutdown():
			# pull next goal off of queue if it's ready
			if not self.goalQueue.empty() and self.webResult.status.status == 3:
				item = self.goalQueue.get()
				self.webFeedback.status.goal_id = item.goal_id
				self.webResult.status.goal_id = item.goal_id
				self.webResult.status.status = 1
				print item

			# sequence everything and publish all data
			self.webFeedback.header.seq+=1
			self.webFeedback_pub.publish(self.webFeedback)
			if abs(item.goal.target_pose.pose.position.x-self.webFeedback.feedback.base_position.pose.position.x)<1:
				print "SUCCESS"
				self.webResult.status.status = 3
				self.webResult_pub.publish(self.webResult)
				self.webResult.header.seq+=1
			elif not self.webResult.status.status == 3:
				#self.outTwist.linear.x = max([-2.0,min([2.0,(poseDist(item.goal.target_pose.pose,self.webFeedback.feedback.base_position.pose))])])
				print "item", item.goal.target_pose.pose
				print "feedback", self.webFeedback.feedback.base_position.pose
				self.outTwist.angular.z = (poseAngle(item.goal.target_pose.pose,self.webFeedback.feedback.base_position.pose))-self.webFeedback.feedback.base_position.pose.orientation.z
				print self.outTwist.angular.z#-self.webFeedback.feedback.base_position.pose.orientation.z
				self.robotCommandPub.publish(self.outTwist)

			rate.sleep()
			

def poseDist(p1, p2):
	dx = p2.position.x-p1.position.x
	dy = p2.position.y-p1.position.y
	return sqrt((dx*dx)+(dy*dy))
def poseAngle(p1, p2):
	dx = p2.position.x-p1.position.x
	dy = p2.position.y-p1.position.y
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

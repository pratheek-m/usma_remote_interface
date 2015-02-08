#!/usr/bin/env python

PACKAGE = 'aribo'
import roslib
roslib.load_manifest(PACKAGE)
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import LaserScan
from math import sin, cos, pi, sqrt, asin, atan2, log


# this is to be turned on for teleop
class obstacleAvoidance:
	def __init__(self):
		self.gain = rospy.get_param('lidar/gain') # defines the gain for the logarithmic function that will define how quickly an object must be avoided
		self.twist_pub = rospy.Publisher('/VC/obstacle',Twist)
		rospy.Subscriber('scan',  LaserScan , self.lidarFeedback)

	def lidarFeedback(self,msg):
                self.outTwist = Twist()
                self.outTwist.linear.x = 0.0
                self.outTwist.linear.y = 0.0
                self.outTwist.linear.z = 0.0
                self.outTwist.angular.x = 0.0 # roll
                self.outTwist.angular.y = 0.0 # pitch
                self.outTwist.angular.z = 0.0 # yaw

		sumForces = [0,0]
		count = 0
		for i in range(0,len(msg.ranges)):
			# 0 rads is front
			curAngle = ((i*msg.angle_increment)+msg.angle_min)
			# only for objects within a meter
			# x and y components
			if msg.ranges[i]<0.75 and curAngle > -pi/2 and curAngle < pi/2:
				count+=1
				sumForces[0] += sin(curAngle)*1/(msg.ranges[i]*msg.ranges[i])
				sumForces[1] += -1*cos(curAngle)*1/(msg.ranges[i]*msg.ranges[i])
		if not count == 0:
			sumForces[0] = sumForces[0] / count
			sumForces[1] = sumForces[1] / count
		print sumForces
		self.outTwist.linear.x = sumForces[1]
		self.outTwist.angular.z = sumForces[0]
		self.twist_pub.publish(self.outTwist)
		
if __name__ == '__main__':
	rospy.init_node('obstacle_avoidance')
	controller = obstacleAvoidance()
	rospy.spin()

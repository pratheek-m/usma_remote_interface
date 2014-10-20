#!/usr/bin/env python

PACKAGE = 'chris_pioneer_control'
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
		self.twist_pub = rospy.Publisher('/OA/Twist',Twist)

	def lidarFeedback(self,msg):
		ouput = Twist()
		sumForces = [0,0]
		for i in range(0,len(msg.ranges)):
			# 0 rads is front
			curAngle = ((i*msg.angle_increment)+msg.angle_min)
			# only for objects within a meter
			# x and y components
			magnitude = -1/msg.ranges[i]
			curForce = [cos(curAngle)*magnitude,sin(curAngle)*magnitude]
			sumForces[0] += curForce[0]
			sumForces[1] += curForce[1]
		# magnitude times direction yields the final result
		output.angular.z = sqrt(sumForces[1]*sumForces[1]+sumForces[0]*sumForces[0])*atan2(sumForces[1],sumForces[0])
		
if __name__ == '__main__':
	rospy.init_node('obstacle_avoidance')
	controller = obstacleAvoidance()
	rospy.Subscriber('/OA/Lidar',  LaserScan , controller.lidarFeedback)
	rospy.spin()

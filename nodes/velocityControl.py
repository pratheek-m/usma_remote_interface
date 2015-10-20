#!/usr/bin/env python

PACKAGE = 'usma_remote_interface'
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
    self.update_freq = rospy.get_param('VC/updateFreq')
    self.sleep_time = 1/self.update_freq
    self.teleopGain = rospy.get_param('VC/teleopGain')
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
    self.alive=True
  
  def teleop(self, data):
  	self.teleopTime = time.time()
  	self.teleopData = data
  
  def stateCb(self, data):
  	self.state = data.data
  
  def obstacle(self,data):
  	self.obstacleData = data
  
  def navCb(self, data):
  	self.navData = data
  
  def updateCommandVelocity(self):
    rate = rospy.Rate(self.update_freq)
    while self.alive == True and not rospy.is_shutdown():
      self.outTwist.linear.x = 0.0
      self.outTwist.angular.z = 0.0
      if self.teleopTime and self.teleopData:
         if (time.time()-self.teleopTime)<1.0:
      	   self.outTwist.linear.x += (self.teleopData.linear.x*self.teleopGain)
      	   self.outTwist.angular.z += (self.teleopData.angular.z*self.teleopGain)
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

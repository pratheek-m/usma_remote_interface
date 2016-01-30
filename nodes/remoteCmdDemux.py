#!/usr/bin/env python

PACKAGE = 'usma_remote_interface'
import roslib
roslib.load_manifest(PACKAGE)
import rospy

from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import String
from math import sin, cos, pi, sqrt, asin, atan2, log
from threading import Thread
import time

# actually controls the velocity based on the multiple inputs provided
class cmdDemux:
  def __init__(self):
    rospy.init_node('usma_remote_cmd_demux', anonymous=True)

    self.update_freq = rospy.get_param('/usma_remote/updateFreq', 10.0)
    self.teleopGain = rospy.get_param('/usma_remote/teleopGain', 1.0)

    self.teleopData = None
    self.teleopTime = None
    rospy.Subscriber('/usma_remote/webcmd', TwistStamped, self.teleop)

    self.topics = []
    self.updateTimes = {}
    self.publishers = {}
    self.curCmds = {}

    self.alive=True
  
  def teleop(self, data):
    if( data.header.frame_id in self.topics ):
        self.updateTimes[data.header.frame_id] = time.time()
        self.curCmds[data.header.frame_id] = data.twist
    else:
        self.topics.append( data.header.frame_id )
        self.updateTimes[data.header.frame_id] = time.time()
        self.curCmds[data.header.frame_id] = data.twist
        self.publishers[data.header.frame_id] = rospy.Publisher( "/usma_remote/"+data.header.frame_id, Twist )

  
  def cmdDemuxControl(self):
    rate = rospy.Rate(self.update_freq)
    while self.alive == True and not rospy.is_shutdown():
      rate.sleep()
      for t in self.topics:
        outTwist = Twist()
        outTwist.linear.x = 0.0
        outTwist.angular.z = 0.0
        if (time.time()-self.updateTimes[t])<(1/self.update_freq * 10.0): # don't publish if we haven't gotten anything for 10 cycles
           outTwist.linear.x +=  (self.curCmds[t].linear.x*self.teleopGain)
           outTwist.angular.z += (self.curCmds[t].angular.z*self.teleopGain)
        self.publishers[t].publish(outTwist)

if __name__ == '__main__':
	try:
		move = cmdDemux()
		t = Thread(target=move.cmdDemuxControl)
		t.start()
		rospy.spin()
		move.alive = False
		t.join()
	except rospy.ROSInterruptException: pass

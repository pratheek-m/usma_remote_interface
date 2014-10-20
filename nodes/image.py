#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import time


class image_converter:

  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    self.image_pub = rospy.Publisher("stream",Image)

    img = cv2.imread('random.jpg')

    self.bridge = CvBridge()
    while not rospy.is_shutdown():
     time.sleep(1)
     try:
       self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
     except CvBridgeError, e:
       print e

def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

#!/usr/bin/env python

import sys
import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

bridge = CvBridge()
def callback(data):
	try:
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		cv2.imshow('image', cv_image)
		cv2.waitKey(10)
		print(cv_image.shape)
	except CvBridgeError as e:
		print(e)


image_sub = rospy.Subscriber("/rexrov/rexrov/camera/camera_image",Image, callback)


def main(args):
  rospy.init_node('target_follower', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)


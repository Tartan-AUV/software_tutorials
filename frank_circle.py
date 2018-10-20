import sys
import cv2

import roslib
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Float32

class go_circle(object):
  
  def __init__(self):
    self.des_vel_pub = rospy.Publisher("/rexrov/cmd_vel", Twist, queue_size = 1)
    self.linear_speed_x = 1.0
    self.angular_speed_x = 1.0

  def circle(self):
    msg = Twist()
    
    msg.linear.x = self.linear_speed_x
    msg.angular.x = self.angular_speed_x

    self.des_vel_pub.publish(msg)

  def execute(self):
    while(not rospy.is_shutdown()):
      self.circle()


def main(args):
  rospy.init_node('circle', anonymous=True)
  s = go_circle()
  s.execute()

if __name__ == '__main__':
  main(sys.argv)





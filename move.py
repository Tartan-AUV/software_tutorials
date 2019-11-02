import sys
import rospy
import roslib
from geometry_msgs.msg import Twist

class circle(object):

	def __init__(self):
		self.pub = rospy.Publisher("/rexrov/cmd_vel", Twist, queue_size=1)
		self.linear_speed_x = 0.4
		self.angular_speed_x = 0.4		

	def spin(self):
		msg = Twist()
		msg.linear.x = self.linear_speed_x
		msg.linear.y = 0.0
		msg.linear.z = 0.0
		msg.angular.x = 0.0
		msg.angular.y = 0.0
		msg.angular.z = self.angular_speed_x
		self.pub.publish(msg)

	def execute(self):
		while(not rospy.is_shutdown()):
			self.spin();

def main(args):
	rospy.init_node('circle', anonymous=True)
	c = circle()
	c.execute();

if __name__ == '__main__':
	main(sys.argv)

import sys
import cv2

import roslib
import rospy

from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int8, Float32

class dice_visual_servoing(object):
	"""
	A class to hit a target dice by controlling the linear speed and yaw speed of
	the submarine in a Gazebo Simulator.
	"""
	def __init__(self):
		# getting the bounding_boxes for Dice D6
		self.bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.callback)
		# sending commands the submarine
		self.des_vel_pub = rospy.Publisher("/rexrov/cmd_vel", Twist, queue_size=1)

		# constansts, how much to change yaw and altitude when hitting the dice
		self.k_yaw = 0.0005
		self.k_alt = 0.0010

		self.idx = None

		# center of target, Bouding Box around Dice 6 in our case.
		self.target_center_x = None
		self.target_center_y = None

		# if the target is detected yet
		self.detected_target = False

		# center of the image
		self.image_center_x = 768/2
		self.image_center_y = 492/2

		# speed to move forward
		self.linear_speed_x = 0.4

	def go_straight(self):
		'''
		Make the sub go straight at linear_speed_x.
		'''
		msg = Twist()

		# going forward
		msg.linear.x = self.linear_speed_x
		# goind left or right.
		msg.linear.y = 0
		# going up or down
		msg.linear.z = 0

		# yaw
		msg.angular.x = 0
		# pitch
		msg.angular.y = 0
		# roll
		msg.angular.z = 0

		# publishing the message to the ros topic.
		# in this case sending the Twist message.
		# which defines how the sub moves.
		self.des_vel_pub.publish(msg)

	def callback(self, msg):
		'''
		Function recieves the messages from the bouding box topic from YOLO
		and processed the message. It sets flag if Dice 6 was detected and
		extracts its center.
		'''
		target_dice = 'D6'
		self.detected_target = False

		for i in range(len(msg.bounding_boxes)):
			if msg.bounding_boxes[i].Class == target_dice:
				self.idx = i
				self.detected_target = True

		if self.detected_target:
			x_min = msg.bounding_boxes[self.idx].xmin
			x_max = msg.bounding_boxes[self.idx].xmax

			y_min = msg.bounding_boxes[self.idx].ymin
			y_max = msg.bounding_boxes[self.idx].ymax
			self.target_center_x = (x_max + x_min)/2
			self.target_center_y = (y_max + y_min)/2
			print(self.target_center_x, self.target_center_y)

	def target_follower(self):
		'''
		This function sends commands to the submarine once we know where the
		dice is. This method of sending commands based on center location
		in the image is called Visual Servoing.
		'''

		msg = Twist()
		# find how far is the center of the target to the center of the image.
		# send command to turn in the direction of the target by small amount.
		d_alt = self.k_alt*(self.image_center_y - self.target_center_y)
		d_yaw = self.k_yaw*(self.image_center_x - self.target_center_x)

		# setting the forward speed, altitude and yaw rate.
		msg.linear.x = self.linear_speed_x
		msg.linear.z = d_alt
		msg.angular.z = d_yaw

		print('Message ')
		print(msg)
		# sending the message for the sub to move.
		self.des_vel_pub.publish(msg)

	def execute(self):
		'''
		This is the main function in the class which makes the submarine
		hit the target.
		'''
		# Basically till we shutdown ROS.
		while(not rospy.is_shutdown()):
			# if we dont detect dice 6, go straight
			if not self.detected_target:
				self.go_straight()
			else:
				# otherwise hit the dice
				print('Hitting Dice')
				self.target_follower()


def main(args):
	# init the ros node.
    rospy.init_node('dice_state', anonymous=True)
	# init the dice visual servoing object
    ds = dice_visual_servoing()
	# execute our mission.
    ds.execute()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

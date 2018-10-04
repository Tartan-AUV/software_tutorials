import sys
import cv2

import roslib
import rospy

from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int8, Float32

bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,callback)
des_vel_pub = rospy.Publisher("/rexrov/cmd_vel", Twist, queue_size=1)
num_sub = rospy.Subscriber("/darknet_ros/found_object",Int8, num_callback)
jerk_sub= rospy.Subscriber("/jerk", Float32, self.jerk_callback )


class dice_visual_servoing(object):
	def __init__(self):
		self.bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.callback)
		self.des_vel_pub = rospy.Publisher("/rexrov/cmd_vel", Twist, queue_size=1)
		self.num_sub = rospy.Subscriber("/darknet_ros/found_object",Int8, self.num_callback)
		self.jerk_sub= rospy.Subscriber("/jerk", Float32, self.jerk_callback )

		self.k_yaw = 0.0005
		self.k_alt = 0.0010

		self.idx = None
        self.center_x = None
        self.center_y = None

        self.detected_target = False

        self.image_center_x = 1280/2
        self.image_center_y = 720/2

        self.YAW_TURN_SPEED = 0.4
		self.k_yaw = 0.0005
		self.k_alt = 0.0010

	def go_down(self):
	    msg = Twist()
	    self.des_vel_pub.publish(msg)

	def go_straight(self):
	    msg = Twist()
	    msg.linear.x = 0.4
	    self.des_vel_pub.publish(msg)

	def go_backward(self):
	    msg = Twist()
	    msg.linear.x = -0.4
	    self.des_vel_pub.publish(msg)

	def turn(self, direction):
		msg = Twist()
		if direction == "left":
			msg.angular.z = self.YAW_TURN_SPEED
		elif direction == "right":
			msg.angular.z = -self.YAW_TURN_SPEED
	    self.des_vel_pub.publish(msg)

	def move_to_side(self, speed):
	    msg = Twist()
	    msg.linear.y = speed
	    self.des_vel_pub.publish(msg)

    def num_callback(self, msg):
		'''
		callback to see how many objects did YOLO detect.
		if detect anything set the flag to True.
		'''
        self.detected_any = False
        if msg.data > 0:
            self.detected_any = True

    def callback(self, msg):
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
            self.center_x = (x_max + x_min)/2
            self.center_y = (y_max + y_min)/2
            print(self.center_x, self.center_y)


	def target_follower(self):
		msg = geometry_msgs.Twist()
		d_alt = self.k_alt*(self.image_center_y - self.center_y)
		d_yaw = self.k_yaw*(self.image_center_x - self.center_x)

		msg.linear.x = self.linear_speed_x
		msg.linear.z = d_alt
		msg.angular.z = d_yaw

		self.des_vel_pub.publish(msg)

	def execute(self):
 		while(not rospy.is_shutdown()):
			self.go_straight()
			if self.detected_target:
				self.target_follower()


def main(args):
    rospy.init_node('dice_state', anonymous=True)
    ds = dice_state()
    ds.execute()
	
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

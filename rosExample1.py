import sys
import cv2

import roslib
import rospy


class moveSubmarineInCircle(object):
	
	def init(self):
		
		self.linearYSpeed = 5
		self.rotatePitch = 5

	'''function that will make the submarine go in circles'''
	#make  U turn with the submarine, turn right 
	def turnAroundSubmarine(self):
	#what is the msg twist?
		msg = Twist()
		
	#to turn right for a u turn, the linear x = 0
		msg.linear.x = 0
		msg.linear.y = self.linearYSpeed
		msg.linear.z = 0

	#yaw
		msg.angular.x = 0

	#pitch changes the pitch to rotate the submarine 
		msg.angular.y = self.rotatePitch

	#roll
		msg.angular.z = 0

		self.des_vel_pub.publish(msg)

#MAIN function that will run the code to turn the submarine around
	def execute(self):
		while (not rospy.is_shutdown()):
			self.turnAroundSubmarine()
		
def main(args):
	#init the ros node
	rospy.init_node("dice_state", anonymous = True)
	submarineCircle = moveSubmarineInCircle()
	submarineCircle.execute()
	try:
		rospy.spin()

	except(KeyboardInterrupt):
		print("shutting Down")


if __name__ == "__main__":
	main(sys.argv)
	

		
		
		

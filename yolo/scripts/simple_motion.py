#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes
import sys

class simple_motion:
	def __init__(self):
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.callback)

	def callback(self, data):
		#rospy.loginfo(data)
		for box in data.bounding_boxes:
			if box.id == 39:
				img_mid = 640
				dif = box.xmax - box.xmin
				bottle_mid = box.xmin + (dif / 2)
				e = 100
				vel = 0.15
				twist = Twist()
				twist.linear.x = 0
				twist.linear.y = 0
				twist.linear.z = 0
				twist.angular.x = 0
				twist.angular.y = 0
				if img_mid > bottle_mid + 100:
					#rospy.loginfo("go left")				
					twist.angular.z = vel
					
				elif img_mid < bottle_mid - 100:
					#rospy.loginfo("go right")
					twist.angular.z = -vel
				else:
					twist.angular.z = 0
				self.pub.publish(twist)			
    
def main(args):
	rospy.init_node('yolo_listener', anonymous=True)
	sm = simple_motion()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("shutting_down")

if __name__ == '__main__':
	main(sys.argv)

#!/usr/bin/env python

import rospy
from std_msgs.msg import String

# Define Talker Function
def talker():
	pub = rospy.Publisher('chatter', String, queue_size = 10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10)

	# ROS main-loop
	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		# Print to screen, write to node's log file, write to rosout
		rospy.loginfo(hello_str)	
		pub.publish(hello_str)
		rate.sleep()

if __name__=="__main__":
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

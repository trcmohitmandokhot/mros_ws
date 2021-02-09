#!/usr/bin/env python

import rospy
from custmsg.msg import camstatus

# Define Talker Function
def talker():
	dashcam_status = camstatus()
	pub = rospy.Publisher('cam_state', camstatus, queue_size = 10)
	rospy.init_node('camnode1_talk', anonymous=True)
	rate = rospy.Rate(10)

	# ROS main-loop
	while not rospy.is_shutdown():
		dashcam_status.cam_name = "tesla"
		dashcam_status.cam_on = True	
		pub.publish(dashcam_status)
		rate.sleep()

if __name__=="__main__":
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

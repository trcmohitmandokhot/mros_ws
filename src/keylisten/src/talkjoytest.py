#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from keylisten.msg import HandheldCont, HandheldInst

# Define Talker Function
def talkjoytest():


	rospy.init_node('talkerjoytest', anonymous=True)


	# Publish a DBW Enabled Message from the Vehicle Platform
	is_dbw_enabled = Bool()
	pub_dbw = rospy.Publisher('/vehicle/dbw_enabled', Bool, queue_size = 10)

	# Establish a publisher for the joy_con.py outputs
	sample_hhcont = HandheldCont()
	sample_hhinst = HandheldInst()
	pub_hhcont = rospy.Publisher('am_ds/joysticks', HandheldCont, queue_size = 1)
	pub_hhinst = rospy.Publisher('am_ds/joybuttons', HandheldInst, queue_size = 1) 

	rate = rospy.Rate(10)
	# ROS main-loop
	while not rospy.is_shutdown():

		is_dbw_enabled = True
		pub_dbw.publish(is_dbw_enabled)
		
		# Assign test values to hhcont message
		sample_hhcont.drive_pedal = 1.0
		sample_hhcont.brake_pedal = 0.0
		sample_hhcont.steering_wheel = -1.0
		sample_hhcont.control_mode = 0
		pub_hhcont.publish(sample_hhcont)

		rate.sleep()

if __name__=="__main__":
	try:
		talkjoytest()
	except rospy.ROSInterruptException:
		pass

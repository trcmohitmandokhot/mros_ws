#!/usr/bin/env python
# Package converts standard ROS joy messages into structure needed by handoff.py node
# Uses custom message structure. Adds dbw_enabled check functionality.
# Version 0.0.2
# Date 02/08/2021
# Anmol Sidhu, Mohit Mandokhot

import rospy
from joy2handoff.msg import JoyCont, JoyInst
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

def joy2hoff_con():
	
	""" Pseudo-code
	# 1. Listen to messages sent by ROS standard joy_node.py. Listen to status of dbw.
	# 2. Understand which joystick is selected
	# 3. Find a map table for joystick used. This will be used to relate button press to actions
	# 4. Convert button press and axes inputs to JoyCont and JoyInst msg outputs
	# 5. Publish the JoyCont and JoyInst messages to topic, which handoff.py can listen to and make relevant actions."""  
	
	rospy.init_node('joy2hoff_con', anonymous=True)

	dbw_enabled = Bool()

	def rec_dbw_enabled(data):
		if data:
			print("1. Platform DBW Enabled")
		else:
			print("2. Platform DBW Disabled")
		dbw_enabled = data

	# Accept data published by ROS joy_node 
	def recd_rosjoyval(data):
		recd_msgbuttons = data.buttons	#int32[]
		recd_msgaxes = data.axes	#float32[]
		# Header data not used
		#print('# of Buttons in DS: ' str(len(recd_msgbuttons)))
		publish_discrete(recd_msgbuttons, dbw_enabled)
		#print('# of Axes in DS: ' str(len(recd_msgaxes)))
		publish_cont(recd_msgaxes)

	# Read Joystick Buttons. Determine content. Publish JoyInst message.
	# 
	# Output four modes - enable, disable, manual, automated
	# Button Maps - buttons[7], buttons[6], buttons[2], buttons[0]. See js_image
	def publish_discrete(data, dbw_state):
		# Recieve data in int32[] list format. Confirm length of list
		len_msg = len(data)
		#print(len_msg)
		sent_msgbuttons = JoyInst()		
		#If pressed (same as 'handoff' enable), set enable - (No status check)
		if data[7] == 1 and (not dbw_state):
		    sent_msgbuttons.enable = True
		    pub_joy_inst.publish(sent_msgbuttons)
		#If pressed, set disable (same as 'handoff' disable)
		if data[6] == 1 and (dbw_state):
		    sent_msgbuttons.disable = True
		    pub_joy_inst.publish(sent_msgbuttons)
		#If pressed, set manual mode (same as 'handoff' Open Loop Mode)
		if data[2] == 1:
		    sent_msgbuttons.manual = True
		    pub_joy_inst.publish(sent_msgbuttons)
		#If pressed, set manual mode (same as 'handoff' auto-test mode)
		if data[0] == 1:
		    sent_msgbuttons.automated = True
		    pub_joy_inst.publish(sent_msgbuttons)

	# Recieve axes data from joystick. Convert to JoyCont() message type
	# Output message left, right, fore, aft
	# Assume Left Stick - Left/Right Control | Right Stick - Fore/Aft Control	
	def publish_cont(data):
		# Recieve data in float32[] list format. Confirm length of list
		len_msg = len(data)
		#print(len_msg)
		sent_msgaxes = JoyCont()
		# Confirm assumption of vehicle joystick map. Assumed same as Xbox
		# Microsoft Xbox 360 Wired Controller for Linux
		# data[0] - Left Stick, L/R Axis | data[1] - Left Stick, U/D Axis
		# data[3] - Right Stick, L/R Axis | data[4] - Right Stick, U/D Axis
		# AStuff Joystick Controller
		# data[1] - Left Stick, L/R Axis | data[0] - Left Stick, U/D Axis
		# data[4] - Right Stick, L/R Axis | data[3] - Right Stick, U/D Axis
		# All values float32 [-1, 1]

		# Left Right Assignment - Left Stick
		if data[1] >= 0:
			sent_msgaxes.left = abs(data[1])
			sent_msgaxes.right = 0
			pub_joy_cont.publish(sent_msgaxes)
		else:
			sent_msgaxes.right = abs(data[1])
			sent_msgaxes.left = 0
			pub_joy_cont.publish(sent_msgaxes)			
		
		# Fore Aft Assignment - Right Stick
		if data[3] < 0:
			sent_msgaxes.aft = abs(data[3])
			sent_msgaxes.fore = 0
			pub_joy_cont.publish(sent_msgaxes)
		else:
			sent_msgaxes.fore = abs(data[3])
			sent_msgaxes.aft = 0
			pub_joy_cont.publish(sent_msgaxes)


	# Listen to DBW Enabled from Vehicle Platform (Car - DBW, Sim - VehSim)
        # dbw_enabled = Bool()
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, rec_dbw_enabled)
	# Define a subscriber to listen to joy_node on "Joy" topic
	ros_joyval = Joy()  
        rospy.Subscriber('/joy', Joy, recd_rosjoyval)


        """ PUBLISHERS """
        # Publish JoyCont and JoyInst messages downstream to handoff
        joy_cont = JoyCont()
	joy_inst = JoyInst()
        pub_joy_inst = rospy.Publisher('/am/joy_inst', JoyInst, queue_size=1)
        pub_joy_cont = rospy.Publisher('/am/joy_cont', JoyCont, queue_size=1)

	""" Publish data based on signals recieved """
	while not rospy.is_shutdown():
		rospy.spin()


if __name__ == '__main__':         
	try:
		joy2hoff_con()
	except rospy.ROSInterruptException:
		pass


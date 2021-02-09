#!/usr/bin/env python
# This node recieves commands from joycon.py and publishes to handoff.py
# This is a simple translator to enable joycon to talk to handoff on the vehicle (real/sim)
# Anmol Sidhu
# Mohit Mandokhot

import rospy
from keylisten.msg import JoyCont, JoyInst, HandheldCont, HandheldInst 
from std_msgs.msg import Bool

def OperatorJoy():
	
        rospy.init_node('operator_joy', anonymous=True)	

	# Recieve continuous data from Joystick
	def rec_joy_hhcont(data):
		publish_cont(data)

	# Read HandheldCont message. Determine content. Publish JoyCont message.
	# HandheldCont contains [float32] drive_pedal, [float32] brake_pedal, [float32] steering_wheel, [unit8] control_mode
	# joy_cont message contains [float 32] x 4 : fore, aft, left, right
	# Simple translation for fore, aft. Left and Right are assigned based on an if condition.
	def publish_cont(data):
		_msgrecjoy = data
		joy_cont.fore = _msgrecjoy.drive_pedal
		joy_cont.aft = _msgrecjoy.brake_pedal		
		if _msgrecjoy.steering_wheel <= 0:
			joy_cont.left = abs(_msgrecjoy.steering_wheel)
			joy_cont.right = 0
		else:
			joy_cont.left = 0
			joy_cont.right = abs(_msgrecjoy.steering_wheel)		
		pub_joy_cont.publish(joy_cont)
		
	# Recieve Handheld instant message
	def rec_joy_hhinst(data):
		publish_discrete(data)

	# Read HandheldInst message. Determine content. Publish JoyInst message.
	# Pass along Enable and Disable
	# home = manual. up = automated
	def publish_discrete(data):
		_msgrecjoy = data
		msg = JoyInst()
		if _msgrecjoy.disable == True:
		    msg.disable = True
		    pub_joy_inst.publish(msg)
		if _msgrecjoy.enable == True:
		    msg.enable = True
		    pub_joy_inst.publish(msg)
		if _msgrecjoy.home == True:
		    msg.mode = 'manual'
		    msg.manual = True
		    pub_joy_inst.publish(msg)
		if _msgrecjoy.up == True:
		    msg.mode = 'automated'
		    msg.automated = True
		    pub_joy_inst.publish(msg)

	# Recieve confirmation from Platform (Real/Sim) on its status. 
	# [MM] - I don't think this function is needed here anymore
	def rec_dbw_enabled(data):
		if data:
			print("1. Platform DBW Enabled")
		else:
			print("2. Platform DBW Disabled")

	""" SUBSCRIBERS """
	# Listen to DBW Enabled from Vehicle Platform (Car - DBW, Sim - VehSim)
        # dbw_enabled = Bool()
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, rec_dbw_enabled)
	# Listen to discrete and continuous messages published by a "joystick".
	joy_contval = HandheldCont()
	rospy.Subscriber('am_ds/joysticks', HandheldCont, rec_joy_hhcont)
	joy_instval = HandheldInst()  
        rospy.Subscriber('am_ds/joybuttons', HandheldInst, rec_joy_hhinst) 

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
	OperatorJoy()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# This node recieves commands from joycon.py and publishes to handoff.py
# This is a simple translator to enable joycon to talk to handoff on the vehicle (real/sim)
# Anmol Sidhu
# Mohit Mandokhot

import sys
import os
import time
import rospy
from keylisten.msg import JoyCont, JoyInst
from keylisten.msg import HandheldCont, HandheldInst
from std_msgs.msg import Bool

class OperatorJoy():
    def __init__(self):

        rospy.init_node('operator_joy', anonymous=True)

        """ SUBSCRIBERS """
	# Listen to DBW Enabled from Vehicle Platform (Car - DBW, Sim - VehSim)
        self.dbw_enabled = Bool()
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.rec_dbw_enabled)
	# Listen to discrete and continuous messages published by a "joystick". Can be anything really.
	self.joy_contval = HandheldCont()
	rospy.Subscriber('am_ds/joysticks', HandheldCont, self.rec_joy_hhcont)
	self.joy_instval = HandheldInst()  
        rospy.Subscriber('am_ds/joybuttons', HandheldInst, self.rec_joy_hhinst) 

        """ PUBLISHERS """
        # Publish JoyCont and JoyInst messages downstream to handoff
        self.joy_cont = JoyCont()
	self.joy_inst = JoyInst()
        self.pub_joy_inst = rospy.Publisher('/am/joy_inst', JoyInst, queue_size=1)
        self.pub_joy_cont = rospy.Publisher('/am/joy_cont', JoyCont, queue_size=1)

	while not rospy.is_shutdown():
		rospy.spin()
		

# Recieve continuous data from Joystick
    def rec_joy_hhcont(self, data):
        self.joy_contval = data

# Read HandheldCont message. Determine content. Publish JoyCont message.
# HandheldCont contains [float32] drive_pedal, [float32] brake_pedal, [float32] steering_wheel, [unit8] control_mode
# joy_cont message contains [float 32] x 4 : fore, aft, left, right
# Simple translation for fore, aft. Left and Right are assigned based on an if condition.

    def publish_cont(self, self.joy_contval):
	_msgrecjoy = self.joy_contval

	self.joy_cont.fore = _msgrecjoy.drive_pedal
	self.joy_cont.aft = _msgrecjoy.brake_pedal
	
	if _msgrecjoy.steering_wheel <= 0:
		self.joy_cont.left = _msgrecjoy.steering_wheel
		self.joy_cont.right = 0
	else
		self.joy_cont.left = 0
		self.joy_cont.right = _msgrecjoy.steering_wheel		

	self.pub_joy_cont.publish(self.joy_cont)
	

    def rec_joy_hhinst(self, data):
        self.joy_instval = data
	data2 = 2*data
	self.publish_discrete.pub(data2)

# Read HandheldInst message. Determine content. Publish JoyInst message.
# Pass along Enable and Disable
# home = manual. up = automated
    def publish_discrete(self, self.data):
	_msgrecjoy = xyz
	
        if _msgrecjoy.disable == True:
            msg = JoyInst()
            msg.disable = True
            self.pub_joy_inst.publish(msg)
        if _msgrecjoy.enable == True:
            msg = JoyInst()
            msg.enable = True
            self.pub_joy_inst.publish(msg)
        if _msgrecjoy.home == True:
            self.mode = 'manual'
            msg = JoyInst()
            msg.manual = True
            self.pub_joy_inst.publish(msg)
        if _msgrecjoy.up == True:
            self.mode = 'automated'
            msg = JoyInst()
            msg.automated = True
            self.pub_joy_inst.publish(msg)

# Recieve confirmation from Platform (Real/Sim) on its status. 
# [MM] - I don't think this function is needed here anymore, as it plays no role for this converter. 

    def rec_dbw_enabled(self, data):
        self.dbw_enabled = data
	if self.dbw_enabled = True:
		print("Platform DBW Enabled")
	else:
		print("Platform DBW Disabled")

if __name__ == '__main__':         
    try:
	OperatorJoy()
    except rospy.ROSInterruptException:
        pass

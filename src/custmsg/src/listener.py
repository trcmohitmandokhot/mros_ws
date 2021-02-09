#!/usr/bin/env python
import rospy
from custmsg.msg import camstatus

# Define Callback Function
def callback(data):
	name = data.cam_name
	state = data.cam_on

	if state:
		print('The node {} is {}.' .format(name, 'ON'))
	else:
		print('The node {} is {}.' .format(name, 'OFF'))
	
# Define Listener, which subscribes to topic sent by Talker
def listener():
	rospy.init_node('camnode1_listen', anonymous=True)
	
	rospy.Subscriber("cam_state", camstatus, callback)

	rospy.spin()

if __name__=="__main__":
	try:
		listener()
	except rospy.ROSInterruptException:
		pass

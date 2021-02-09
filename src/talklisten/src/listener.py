#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# Define Callback Function
def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

# Define Listener, which subscribes to topic sent by Talker
def listener():
	rospy.init_node('listener', anonymous=True)
	
	rospy.Subscriber("chatter", String, callback)

	rospy.spin()

if __name__=="__main__":
	listener()

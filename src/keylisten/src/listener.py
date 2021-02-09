#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Define Callback Function
def callback(data):
	rospy.loginfo("Received a key_vel message!")	
	rospy.loginfo("Linear Velocity [x,y,z] = [%f, %f, %f]"%(data.linear.x, data.linear.y, data.linear.z))
	

# Define Listener, which subscribes to topic sent by Talker
def listener():
	rospy.init_node('listener_key_vel', anonymous=True)
	
	rospy.Subscriber("key_vel", Twist, callback)

	rospy.spin()

if __name__=="__main__":
	listener()

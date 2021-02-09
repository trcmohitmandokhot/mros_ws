#!/usr/bin/env python

# Authors:
# Anmol Sidhu
# Mohit Mandokhot

# The purpose is to provide a simulated vehicle to aid planner code dev to work with Autoware
# This node simulates a simple vehicle that can be driven around with keyboard
# The vehicle takes speed, accel, decel, curvature, curvature rate as inputs (like SCC)
# The node outputs vehicle pose (as ndt_pose) and /vehicle/twist like (like DBW)
# The node also subscribes to rviz initial pose to reset position if necessary
# The vehicle model works in SAE but Autoware is ISO - watch out for transforms

from lib_vehicle import Interp, Rate_Limit, Vehicle_Sim
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
import time
import math
import numpy as np
from tf.transformations import *

class VehSim():
    def __init__(self):
        rospy.init_node('vehsim', anonymous=True)

        """ Variables """
        self.tnow = 0.0
        self.VX_MAX = 10.0
        self.ACCEL_MAX = 1.0
        self.DECEL_MAX = 4.0
        self.COAST = 0.25
        self.CURVATURE_MAX = 0.15
        self.CURVATURE_RATE = 0.18
        self.POSE_INIT = [10.0, 10.0, 45] # east (m), north (m), heading (deg)
        self.driver_curvature = 0.0
        self.tcomm_driver = 0.0

        # Vehicle Object
        self.vehicle_sim = Vehicle_Sim() 
        self.vehicle_sim.init(self.POSE_INIT[0], self.POSE_INIT[1], self.POSE_INIT[2]) # east, north, heading
        
        """ PUBLISHERS """
        # Vehicle Twist
        self.vehicle_twist = Twist()
        self.pub_vehicle_twist = rospy.Publisher('/vehicle/twist', Twist, queue_size=1)
        # vehicle Pose
        self.vehicle_pose = PoseStamped()
        self.pub_vehicle_pose = rospy.Publisher('/ndt_pose', PoseStamped, queue_size=1)
        
        """ SUBSCRIBERS """
        # Keyboard Teleop
        self.key_cmd = Twist()
        rospy.Subscriber('/key_vel', Twist, self.rec_key_cmd)
        # RVIZ Initial Pose
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.rec_init_pose)
        # Curvature from driver
        rospy.Subscriber('/am_curvature', Float32, self.rec_driver_curvature)

        self.t0 = time.time()
        """ ROS Loop  """
        rate = rospy.Rate(40) # Hz
        while not rospy.is_shutdown():
            self.run()
            rate.sleep()

    # Receive driver curvature
    def rec_driver_curvature(self, data):
        self.driver_curvature = data.data
        self.tcomm_driver = self.tnow

    # Receive keyboard teleop
    def rec_key_cmd(self, data):
        self.key_cmd = data

    # Receive init pose from rviz
    def rec_init_pose(self, data): 
        # get quaternion list
        veh_quat = [0.0, 0.0, 0.0, 0.0]
        veh_quat[0] = data.pose.pose.orientation.x
        veh_quat[1] = data.pose.pose.orientation.y
        veh_quat[2] = data.pose.pose.orientation.z
        veh_quat[3] = data.pose.pose.orientation.w
        # convert quaternion to euler and subtract from 90 (XY to EN)
        heading = 90-euler_from_quaternion(veh_quat)[2]*180/np.pi # convert heading to range [0 to 360]
        if heading < 0.0:
            heading = 360+heading
        # reinitialize the vehicle
        self.vehicle_sim.init(data.pose.pose.position.x, data.pose.pose.position.y, heading) # east, north, heading

    # Main run
    def run(self):
        self.tnow = time.time() - self.t0

        ##########################################################
        # Keyboard inputs to vehicle speed and curvature commands
        # Linear
        if self.key_cmd.linear.x == 1.0:
            vel_set = self.VX_MAX
            decel_set = self.DECEL_MAX
        elif self.key_cmd.linear.x == -1.0:
            vel_set = 0.0
            decel_set = self.DECEL_MAX
        else:
            vel_set = 0.0
            decel_set = self.COAST
        # Angular
        if self.tnow - self.tcomm_driver > 1.0: # connection with driver lost
            curvature_set = -self.key_cmd.angular.z*self.CURVATURE_MAX # (-ve to convert key direction from ISO to SAE for vehicle)
        else:
            curvature_set = -self.driver_curvature

        ####################
        # Run vehicle model
        self.vehicle_sim.run(vel_set, self.ACCEL_MAX, decel_set, curvature_set, self.CURVATURE_RATE)
        
        ################
        # Assign topics
        # Vehicle twist
        self.vehicle_twist.linear.x = self.vehicle_sim.vx # speed in m/s
        self.vehicle_twist.angular.z = -self.vehicle_sim.yr*np.pi/180 # yawrate in rad/s (-ve to convert vehicle SAE to ISO XY)
        # Vehicle Pose
        self.vehicle_pose.header.frame_id = "/map"
        self.vehicle_pose.pose.position.x = self.vehicle_sim.east # assign east as x in meters
        self.vehicle_pose.pose.position.y = self.vehicle_sim.north # assign north as y in meters
        yaw_xy = 90 - self.vehicle_sim.yaw # yaw in XY frame (angle with x) is 90-yaw in EN frame (which is angle with north)
        veh_quat = quaternion_from_euler(0, 0, yaw_xy*np.pi/180) # convert euler to quaternion for rviz, yaw in rad from north
        self.vehicle_pose.pose.orientation.z = veh_quat[2]
        self.vehicle_pose.pose.orientation.w = veh_quat[3]
        
        #################
        # Publish topics
        self.pub_vehicle_twist.publish(self.vehicle_twist)
        self.pub_vehicle_pose.publish(self.vehicle_pose)

if __name__ == '__main__':
    try:
        VehSim()
    except rospy.ROSInterruptException:
        pass

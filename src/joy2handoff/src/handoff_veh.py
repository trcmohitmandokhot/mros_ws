#!/usr/bin/env python

# Old Handoff Structure. Written by Anmol Sidhu for AVAW3.0
# Copied for reference to understand vehicle joystick structure. 

from library_sim import Interp, Rate_Limit
#Vehicle_Sim,  Vehicle_Speed_Controller
from library_joycon import *

import rospy
import rosparam
from std_msgs.msg import Empty, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped, PoseWithCovarianceStamped
#from dbw_mkz_msgs.msg import BrakeCmd, ThrottleCmd, SteeringCmd, WheelSpeedReport
from automotive_platform_msgs.msg import SpeedMode, SteerMode, GearCommand, TurnSignalCommand
from am_ds.msg import HandheldCont, HandheldInst
import time
import numpy as np
import math

class Handoff():
    def __init__(self):
        rospy.init_node('handoff', anonymous=True)

        """ PARAMS """
        global ENABLE_LON, ENABLE_LAT, SPEED_INC, SPEED_MAX, ACCEL_MAX, DECEL_MAX, AX_COAST, AY_MAX, CURVATURE_MAX, CURVATURE_RATE
        global DRIVE_PEDAL_RATE, BRAKE_PEDAL_RATE, BRAKE_HOLD, STEER_RATE, STEER_LIMIT
        global JOY_BRAKE_DB, JOY_BRAKE_MAX, JOY_DRIVE_DB, JOY_DRIVE_MAX
        global INIT_POSE_X, INIT_POSE_Y, INIT_POSE_H, INIT_POSE_W

        # Load with launch file
        ENABLE_LON = rospy.get_param("/am_ds_handoff/ENABLE_LON")
        ENABLE_LAT = rospy.get_param("/am_ds_handoff/ENABLE_LAT")
        SPEED_INC = rospy.get_param("/am_ds_handoff/SPEED_INC")
        SPEED_MAX = rospy.get_param("/am_ds_handoff/SPEED_MAX")
        ACCEL_MAX = rospy.get_param("/am_ds_handoff/ACCEL_MAX")
        DECEL_MAX = rospy.get_param("/am_ds_handoff/DECEL_MAX")
        DRIVE_PEDAL_RATE = rospy.get_param("/am_ds_handoff/DRIVE_PEDAL_RATE")
        BRAKE_PEDAL_RATE = rospy.get_param("/am_ds_handoff/BRAKE_PEDAL_RATE")
        STEER_RATE = rospy.get_param("/am_ds_handoff/STEER_RATE")
        STEER_LIMIT = rospy.get_param("/am_ds_handoff/STEER_LIMIT")
        BRAKE_HOLD = rospy.get_param("/am_ds_handoff/BRAKE_HOLD")
        JOY_BRAKE_DB = rospy.get_param("/am_ds_handoff/JOY_BRAKE_DB")
        JOY_BRAKE_MAX = rospy.get_param("/am_ds_handoff/JOY_BRAKE_MAX")
        JOY_DRIVE_DB = rospy.get_param("/am_ds_handoff/JOY_DRIVE_DB")
        JOY_DRIVE_MAX = rospy.get_param("/am_ds_handoff/JOY_DRIVE_MAX")
        INIT_POSE_X = rospy.get_param("/am_ds_handoff/INIT_POSE_X")
        INIT_POSE_Y = rospy.get_param("/am_ds_handoff/INIT_POSE_Y")
        INIT_POSE_H = rospy.get_param("/am_ds_handoff/INIT_POSE_H")
        INIT_POSE_W = rospy.get_param("/am_ds_handoff/INIT_POSE_W")
        AX_COAST = rospy.get_param("/am_ds_handoff/AX_COAST")
        AY_MAX = rospy.get_param("/am_ds_handoff/AY_MAX")
        CURVATURE_MAX = rospy.get_param("/am_ds_handoff/CURVATURE_MAX")
        CURVATURE_RATE = rospy.get_param("/am_ds_handoff/CURVATURE_RATE")

        """ Variables """
        # Generic interp
        #self.signal_data = "0,2,4,6,1000 % 0,0.25,0.25,0,0"
        #self.sig_interp1 = Interp() 
        #self.sig_interp1.init(self.signal_data)
    
        self.t0 = time.time()
        self.tnow = 0.0
        self.enabled = False
        self.control_mode = 0
        self.speed = 0.0
        self.speed_set = 0.0
        self.speed_latch = 0.0
        self.yawrate_set = 0.0
        self.steer_set = 0.0
        self.curvature_wheel_set = 0.0
        self.steer_target = 0.0
        self.speed_target = 0.0
        self.drive_pedal_set = 0.0
        #self.drive_pedal_target = 0.0
        self.brake_pedal_set = 0.0
        #self.brake_pedal_target = 0.0
        self.speed_set_buttons = 0
        self.pose_reset = False
        self.ssc_mode = 0
        self.accel_set = ACCEL_MAX
        self.comm_joystrL = False
        self.comm_joystrR = False

        # Load in script
        #data = rosparam.load_file("/home/oryx/amas_ws/src/am_ds/config/params.yaml")
        #number_float = data[0][0]['number_float']
        #print(number_float)

        """ PROFILE OBJECTS """
        self.speedRateLim = Rate_Limit()
        self.steerRateLim = Rate_Limit()
        #self.drivePedalRateLim = Rate_Limit()
        #self.brakePedalRateLim = Rate_Limit()
        #self.speedController = Vehicle_Speed_Controller()

        """ PUBLISHERS """
        # Enable DBW
        self.pub_enable = rospy.Publisher('vehicle/enable', Empty, queue_size=1) 
        # Disable DBW
        self.pub_disable = rospy.Publisher('vehicle/disable', Empty, queue_size=1) 
        # Twist Target
        self.twist_target = Twist()
        #self.pub_twist_target = rospy.Publisher('am_ds/twist_target', Twist, queue_size=1)

        # Initial pose
        self.initialpose = PoseWithCovarianceStamped()
        self.pub_initialpose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.speed_mode = SpeedMode() 
        self.pub_speed_mode = rospy.Publisher('/ssc/arbitrated_speed_commands', SpeedMode, queue_size=1)
        self.speed_mode.acceleration_limit = ACCEL_MAX
        self.speed_mode.deceleration_limit = DECEL_MAX
        self.steer_mode = SteerMode()
        self.pub_steer_mode = rospy.Publisher('/ssc/arbitrated_steering_commands', SteerMode, queue_size=1)

        self.gear_cmd = GearCommand()
        self.pub_gear_cmd = rospy.Publisher('/ssc/gear_select', GearCommand, queue_size=1)

        self.turn_cmd = TurnSignalCommand()
        self.pub_turn_cmd = rospy.Publisher('/ssc/turn_signal_command', TurnSignalCommand, queue_size=1)

        """ SUBSCRIBERS """

	    # PS4 Joystick
        self.joyps4 = Joy()
        rospy.Subscriber('/joy', Joy, self.rec_joyps4)

        # Joycon sticks
        #self.joysticks = HandheldCont()
        #rospy.Subscriber('am_ds/joysticks', HandheldCont, self.rec_joysticks)
        #rospy.Subscriber('am_ds/vel', Twist, self.rec_vel)
        rospy.Subscriber('vehicle/twist', TwistStamped, self.rec_vel_stamped)
        #rospy.Subscriber('vehicle/wheel_speed_report', WheelSpeedReport, self.rec_wheel_speed)
        self.twist_cmd = Twist()
        #rospy.Subscriber('am_ds/twist_cmd', Twist, self.rec_twist_cmd)
        rospy.Subscriber('twist_cmd', TwistStamped, self.rec_twist_cmd_stamped)
        self.dbw_enabled = Bool()
        rospy.Subscriber('vehicle/dbw_enabled', Bool, self.rec_dbw_enabled)
        self.twist_target_wp = Twist()
        rospy.Subscriber('am_ds/twist_wp', Twist, self.rec_twist_wp)

        """ ROS Loop """
        rate = rospy.Rate(50) # Hz
        while not rospy.is_shutdown():
            self.run()
            rate.sleep()

    def rec_joyps4(self, data):  
        #print(data.buttons)
        if not self.pose_reset and data.buttons[9] == 1 and data.buttons[10] == 1: # both sticks pressed
            self.pose_reset = True
            #print("publish pose")
            self.initialpose.header.frame_id = "world"
            self.initialpose.pose.pose.position.x = INIT_POSE_X
            self.initialpose.pose.pose.position.y = INIT_POSE_Y
            self.initialpose.pose.pose.position.z = 0.0
            self.initialpose.pose.pose.orientation.x = 0.0
            self.initialpose.pose.pose.orientation.y = 0.0
            self.initialpose.pose.pose.orientation.z = INIT_POSE_H
            self.initialpose.pose.pose.orientation.w = INIT_POSE_W
            self.initialpose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]
            self.pub_initialpose.publish(self.initialpose)

        temp = max(data.axes[4], 0.0)
        self.drive_pedal_set = temp
        temp = -min(data.axes[4], 0.0)
        self.brake_pedal_set = temp
        tempL = min((data.axes[2] - 1.0)/2.0, 0.0)
        if tempL == 0.0:
            self.comm_joystrL = True
        tempR = min((data.axes[5] - 1.0)/2.0, 0.0)
        if tempR == 0.0:
            self.comm_joystrR = True
        if self.comm_joystrL and self.comm_joystrR:
            self.curvature_wheel_set = tempR - tempL
        #print(self.curvature_wheel_set)
        #print("joy", self.drive_pedal_set, self.brake_pedal_set, self.curvature_wheel_set)

        if data.buttons[7] == 1 and not self.enabled:
            self.send_enable_dbw()
            self.ssc_mode = 1

        if data.buttons[6] == 1 and self.enabled:
            rospy.loginfo("Disable DBW")
            self.ssc_mode = 0
            self.pub_disable.publish()
        
        if data.buttons[2] == 1:
            self.control_mode = 1 # X openloop	
            rospy.loginfo("Open-loop Mode")

        if data.buttons[1] == 1:
            self.control_mode = 4 # Stop (quick decel to zero)
            rospy.loginfo("Hold Brake")

        if data.buttons[3] == 1:
            self.control_mode = 2 # speed control
            self.speed_latch = self.speed
            rospy.loginfo("Speed Mode: Setpoint = " + str(self.speed_set))

        if data.buttons[0] == 1:
            self.control_mode = 3 # auto-test mode
            rospy.loginfo("Auto-Run Mode")

        if data.buttons[5] == 1 and self.speed_set_buttons == 0:
            self.speed_set_buttons = 1
            if self.control_mode == 2:
                self.speed_latch = min(self.speed_latch + SPEED_INC, SPEED_MAX)
                rospy.loginfo("Increment Speed: Setpoint = " + str(self.speed_latch))

        if data.buttons[4] == 1 and self.speed_set_buttons == 0:
            self.speed_set_buttons = 1
            if self.control_mode == 2:
                self.speed_latch = max(self.speed_latch - SPEED_INC, 0.0)
                rospy.loginfo("Decrement Speed: Setpoint = " + str(self.speed_latch))
	
        if data.buttons[4] == 0 and data.buttons[5] == 0: # released speed set 
            self.speed_set_buttons = 0

        if data.buttons[9] == 0 or data.buttons[10] == 0: # released pose reset
            self.pose_reset = False


    def send_enable_dbw(self):
        rospy.loginfo("Enable DBW")
        #self.drive_cmd.clear = True
        #self.pub_drive_cmd.publish(self.drive_cmd)
        #time.sleep(0.1)
        #self.drive_cmd.clear = False
        #self.pub_brake_cmd.publish(self.brake_cmd)
        self.pub_enable.publish()

    def rec_twist_wp(self, data):
        self.twist_target_wp = data

    def rec_vel(self, data):
        self.speed = data.linear.x

    def rec_vel_stamped(self, data):
        self.speed = data.twist.linear.x

    def rec_dbw_enabled(self, data):
        self.enabled = data.data
        if not self.enabled:
            self.control_mode = 0

    def rec_wheel_speed(self, data):
        self.speed = 1.183*5.0/18.0*0.5*(data.rear_left + data.rear_right)

    def rec_twist_cmd(self, data):
        self.speed_set = data.linear.x
        #print(self.speed_set)
        self.yawrate_set = data.angular.z
        #print(self.yawrate_set)

    def rec_twist_cmd_stamped(self, data):
        self.speed_set = data.twist.linear.x
        #print(self.speed_set)
        self.yawrate_set = data.twist.angular.z
        #print(self.yawrate_set)

    '''def twist_to_steer(self, speed_cmd, yawrate_cmd, speed):
        #L = 2.7
        #Ksr = 20.0
        #Kus = 3.0/9.81
        #AY_MAX = 4.0
        SR = 15
        Wheelbase = 2.7
        KUS = 3.0
        V_MIN = 0.1
        G = 9.81
        v = max(speed, V_MIN)
        direction = math.copysign(1, yawrate_cmd)
        #curvature = abs(yawrate_cmd)/max(speed_cmd, 1) # min speed to consider 1 m/s
        #curvature_ = min(curvature, 1.0/8.0) # max curvature to consider 1/8 m
        #ay = curvature_*speed**2
        #if ay > 4.0: # max ay 4 m/s^2
            #ay = 4.0
            #curvature_ = ay/speed**2
        #steer = dir*min((Ksr*L*curvature_ + Kus*ay)*180/np.pi, STEER_LIMIT)
        ay = min(abs(speed_cmd*yawrate_cmd), AY_MAX)
        _R_MAX = STEER_LIMIT*math.pi/180/SR/Wheelbase

        _r_cmd = abs(yawrate_cmd/max(speed_cmd, V_MIN))
        ay_req = min(_r_cmd*v**2, AY_MAX)
        _r = min(ay_req/v**2, _R_MAX)
        
        steer_out = direction*min((_r*Wheelbase*180/math.pi + KUS/G*ay)*SR, STEER_LIMIT)
        #print("ayr", _r_cmd)

        return steer_out'''

    def twist_to_curvature(self, speed_cmd, yawrate_cmd, speed):
        V_MIN = 1.0
        v = max(speed, V_MIN)
        curvature_cmd = abs(yawrate_cmd/max(speed_cmd, V_MIN))
        ay_req = min(curvature_cmd*v**2, AY_MAX)
        curvature = min(ay_req/v**2, CURVATURE_MAX)*math.copysign(1, yawrate_cmd)
        return curvature

    '''def scale_steer_speed(self, speed):
        #HWA_LIM = 450
        AY_MAX = 4.0
        SR = 20.0
        Wheelbase = 2.7
        KUS = 3.0
        V_MIN = 1.0
        G = 9.81
        v = max(speed, V_MIN)
        _R_MAX = STEER_LIMIT*math.pi/180/SR/Wheelbase
        _r = min(AY_MAX/v**2, _R_MAX)
        #steer[i] = _r*Wheelbase*SR*180/math.pi
        return min((_r*Wheelbase*180/math.pi + KUS/G*_r*speed**2)*SR, STEER_LIMIT)'''

    def scale_curvature_speed(self, speed):
        V_MIN = 1.0
        v = max(speed, V_MIN)
        curvature = min(AY_MAX/v**2, CURVATURE_MAX)
        return curvature

    def rec_joysticks(self, data):
        if self.control_mode != 1 and data.control_mode == 1: # mode set to openloop
            if self.enabled:
                rospy.loginfo("Open-loop Mode")
                self.control_mode = 1
        elif self.control_mode != 2 and data.control_mode == 2: # mode set to speed
            if self.enabled:
                self.control_mode = 2
                self.speed_set = self.speed
                rospy.loginfo("Speed Mode: Setpoint = " + str(self.speed_set))
        elif self.control_mode != 3 and data.control_mode == 3: # mode set to auto
            if self.enabled:
                self.control_mode = 3
                rospy.loginfo("Auto-Run Mode")
        elif self.control_mode != 4 and data.control_mode == 4: # mode set to hold
            if self.enabled:
                self.control_mode = 4
                rospy.loginfo("Hold Brake Mode")
        self.joysticks = data # update all sticks

    def apply_db(self, input, stroke, db, saturation):
        return max(input-db, 0.0) * stroke/(stroke-db) * saturation

    def run(self):  
        self.tnow = time.time() - self.t0

        # Yaw Rate Command Processing
        if self.control_mode == 3:
            self.steer_set = self.twist_to_curvature(self.speed_set, self.yawrate_set, self.speed)
        # All modes curvature commnand processing
        else:
            self.steer_set = self.scale_curvature_speed(self.speed) * self.curvature_wheel_set
            #print(self.steer_set, self.curvature_wheel_set)
        # Apply Steer Rate Limit
        self.steerRateLim.run(0.0, CURVATURE_RATE, self.steer_set)
        self.steer_target = self.steerRateLim.y

        #self.pub_gear_cmd.publish(self.gear_cmd)
        #self.pub_turn_cmd.publish(self.turn_cmd)
               
        # Mode Openloop
        if self.control_mode == 1:
            temp1 = self.apply_db(self.drive_pedal_set, 1.0, 0.1, 1.0)
            temp2 = self.apply_db(self.brake_pedal_set, 1.0, 0.1, 1.0)
            if temp1 == 0 and temp2 == 0:
                self.accel_set = AX_COAST
                self.speed_set = 0.0
            elif temp1 > 0.0:
                self.accel_set = temp1*ACCEL_MAX
                self.speed_set = SPEED_MAX
            else:
                self.accel_set = temp2*DECEL_MAX
                self.speed_set = 0.0

        # Mode Set Speed
        if self.control_mode == 2:
            self.accel_set = ACCEL_MAX
            self.speed_set = self.speed_latch

        # Mode Quick stop
        if self.control_mode == 4:
            self.accel_set = DECEL_MAX
            self.speed_set = 0.0

        # Apply Speed Rate Limit
        if self.control_mode == 3:
            self.speedRateLim.run(self.speed, self.accel_set, self.twist_target_wp.linear.x)
            print("A mode", self.twist_target_wp.linear.x)
        else:
            self.speedRateLim.run(self.speed, self.accel_set, self.speed_set)
        self.speed_target = self.speedRateLim.y

        # Final SSC commands
        self.twist_target.linear.x = self.speed_target
        self.speed_mode.speed = self.speed_target
        self.speed_mode.mode = self.ssc_mode
        self.speed_mode.header.stamp = rospy.Time.now()
        self.pub_speed_mode.publish(self.speed_mode)
        
        self.steer_mode.mode = self.ssc_mode
        self.steer_mode.curvature = self.steer_target
        self.steer_mode.max_curvature_rate = CURVATURE_RATE
        self.steer_mode.header.stamp = rospy.Time.now()
        self.pub_steer_mode.publish(self.steer_mode)
        
        #print(self.control_mode, self.speed_target, self.steer_target)     
        
if __name__ == '__main__':
    try:
        Handoff()
    except rospy.ROSInterruptException:
        pass

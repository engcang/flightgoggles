#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 02:02:03 2019

@author: mason
"""

''' import libraries '''
import time
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from math import pow,atan2,sqrt,sin,cos
import numpy as np

import rospy
from mav_msgs.msg import RateThrust
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

import tf

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


global joy_check
joy_check=0
global first
first = 0
global g
global max_ang_x
global max_ang_y

g = 9.81
max_ang_x = 30 * np.pi/180 # max angle -> edit here to change velocity of vehicle
max_ang_y = 30 * np.pi/180

''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.local_deg_pub = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size=10)
        self.reset_pub = rospy.Publisher('/uav/collision', Empty, queue_size=1)
        self.tf_pos_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        self.joy_sub = rospy.Subscriber('/control_nodes/joy', Joy, self.joy_callback)

	self.br = tf.TransformBroadcaster()
	self.count=1
	self.lpf_count=1
	self.prev_time=rospy.Time.now()
	self.mean_linear_vel=0
	self.max_linear_vel=0
	self.mean_ang_vel=0
	self.max_ang_vel=0
	self.length=0

        self.rate = rospy.Rate(30)

        self.deg = RateThrust()
        self.joy = Joy()
        self.reset = Empty()
        self.mode = 2 #default is mode 2

    def tf_callback(self, msg):
	global first
        for i in range(0,len(msg.transforms)):
		if msg.transforms[i].child_frame_id=="uav/imu":
		    self.truth=msg.transforms[i].transform.translation
		    orientation_list = [msg.transforms[i].transform.rotation.x, msg.transforms[i].transform.rotation.y, msg.transforms[i].transform.rotation.z, msg.transforms[i].transform.rotation.w]
		    (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
		    #self.br.sendTransform((23+self.truth.y, 18-self.truth.x, self.truth.z-4.3),quaternion_from_euler(self.roll,self.pitch,self.yaw-np.pi/2),rospy.Time.now(),"uav/imu_check","world")
		    self.br.sendTransform((self.truth.x, self.truth.y, self.truth.z-1),quaternion_from_euler(self.roll,self.pitch,self.yaw),rospy.Time.now(),"uav/imu_check","world")
		if msg.transforms[i].child_frame_id=="uav/imu_check":
			if first==0:
				self.prev_pos = msg.transforms[i].transform.translation
				self.prev_rot = msg.transforms[i].transform.rotation
				first = 1
			else : 
				self.current_time=msg.transforms[i].header.stamp
				self.delta_time = (self.current_time-self.prev_time)/rospy.Duration(1)
				self.current_pos=msg.transforms[i].transform.translation
				self.current_rot=msg.transforms[i].transform.rotation
				if self.lpf_count%15==0:
					(p_roll, p_pitch, p_yaw) = euler_from_quaternion([self.prev_rot.x,self.prev_rot.y,self.prev_rot.z,self.prev_rot.w])
					(c_roll, c_pitch, c_yaw) = euler_from_quaternion([self.current_rot.x,self.current_rot.y,self.current_rot.z,self.current_rot.w])
					current_length=sqrt(pow(self.current_pos.x-self.prev_pos.x,2)+pow(self.current_pos.y-self.prev_pos.y,2)+pow(self.current_pos.z-self.prev_pos.z,2))
					self.current_linear_vel=current_length/float(self.delta_time)
					roll_def=c_roll-p_roll
					pitch_def=c_pitch-p_pitch
					yaw_def=c_yaw-p_yaw
					if roll_def>np.pi:
						roll_def-= 2*np.pi
					if roll_def<-np.pi:
						roll_def+= 2*np.pi
					if pitch_def>np.pi:
						pitch_def-= 2*np.pi
					if pitch_def<-np.pi:
						roll_def+= 2*np.pi
					if yaw_def>np.pi:
						yaw_def-= 2*np.pi
					if yaw_def<-np.pi:
						yaw_def+= 2*np.pi
					self.current_angular_vel=sqrt(pow(roll_def,2)+pow(pitch_def,2)+pow(yaw_def,2))/float(self.delta_time)
					if not self.current_linear_vel==0:
						self.length+=current_length
						self.mean_linear_vel=self.mean_linear_vel/self.count*(self.count-1)+self.current_linear_vel/self.count
						self.mean_ang_vel=self.mean_ang_vel/self.count*(self.count-1)+self.current_angular_vel/self.count
						if self.current_linear_vel>self.max_linear_vel:
							self.max_linear_vel=self.current_linear_vel
						if self.current_angular_vel>self.max_ang_vel:
							self.max_ang_vel=self.current_angular_vel
						self.count+=1
					self.prev_pos=self.current_pos
					self.prev_rot=self.current_rot
					self.prev_time=self.current_time
					print("Total Path length : %.3f"%self.length)
					print("Linear vel-curr : %.3f, mean : %.3f, max : %.3f"%(self.current_linear_vel, self.mean_linear_vel, self.max_linear_vel))
					print("Angular vel-curr : %.3f, mean : %.3f, max : %.3f\n\n"%(self.current_angular_vel, self.mean_ang_vel, self.max_ang_vel))
				self.lpf_count+=1

    def joy_callback(self, msg):
        self.joy = msg
        global joy_check
        if len(self.joy.axes)>0 or len(self.joy.buttons)>0 :
            joy_check=1
            if self.joy.buttons[4]==1:
                self.mode=1
            if self.joy.buttons[5]==1:
                self.mode=2
            if self.joy.buttons[0]==1:
                self.reset_pub.publish(self.reset)

def input(rbt):
    global max_ang_x
    global max_ang_y
    global g

##Mode 2, default
#joy_axes: {pitch: 4, roll: 3, yaw: 0, vertical: 1}
    if rbt.mode==2:
        rbt.deg.angular_rates.x=2*(-rbt.joy.axes[3]*max_ang_x - rbt.roll)
        rbt.deg.angular_rates.y=2*( rbt.joy.axes[4]*max_ang_y - rbt.pitch)
        rbt.deg.angular_rates.z=1.5*(rbt.joy.axes[0])
        rbt.deg.thrust.z = g + rbt.joy.axes[1]*g #throttle
##Mode 1
#joy_axes: {pitch: 1, roll: 3, yaw: 0, vertical: 4}
    elif rbt.mode==1:
        rbt.deg.angular_rates.x=2*(-rbt.joy.axes[3]*max_ang_x - rbt.roll)
        rbt.deg.angular_rates.y=2*( rbt.joy.axes[1]*max_ang_y - rbt.pitch)
        rbt.deg.angular_rates.z=1.5*(rbt.joy.axes[0])
        rbt.deg.thrust.z = g + rbt.joy.axes[4]*g #throttle

### removing quivering ###
#    if abs(rbt.deg.angular_rates.x) < 0.08:
#        rbt.deg.angular_rates.x=0
#    if abs(rbt.deg.angular_rates.y) < 0.08:
#        rbt.deg.angular_rates.y=0
#    if abs(rbt.deg.angular_rates.z) < 0.08:
#        rbt.deg.angular_rates.z=0    

#    print("Mode < %d > now, press L1 or R1 to change, Press Button[0] to reset Simulator"%rbt.mode)
#    print("Input : X: %.3f  Y: %.3f  Z: %.3f  Throttle: %.3f"%(rbt.deg.angular_rates.x, rbt.deg.angular_rates.y, rbt.deg.angular_rates.z, rbt.deg.thrust.z))
#    print("Angle(Degree): roll: %.4f pitch: %.4f yaw: %.4f \n"%(rbt.roll/np.pi*180, rbt.pitch/np.pi*180, rbt.yaw/np.pi*180)) #radian : +-pi

    rbt.deg.header.stamp = rospy.Time.now()
    rbt.local_deg_pub.publish(rbt.deg)

##############################################################################################

alpha = robot()
alpha.joy_callback(alpha.joy)
time.sleep(1) #wait 1 second to assure that all data comes in

''' main '''
if __name__ == '__main__':
    while 1:
        try:
            if joy_check==1:
                input(alpha)
                alpha.rate.sleep()
            else: 
                alpha.rate.sleep()
                pass
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass

#! /usr/bin/python
# -*- coding: utf-8 -*-


# M3 -- Meka Robotics Robot Components
# Copyright (C) 2010 Meka Robotics
# Author: edsinger@mekabot.com (Aaron Edsinger)

# M3 is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# M3 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with M3.  If not, see <http://www.gnu.org/licenses/>.
import m3
import time
import os
#import roslib; roslib.load_manifest('meka_description')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import m3.nanokontrol as m3k
import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.humanoid as m3h
import m3.omnibase as m3o
import m3.toolbox as m3t
import m3.joint_zlift as m3z

import m3.component_factory as m3f
import time
import math
import numpy as npy
import PyKDL as kdl

def get_zlift(proxy):
    assert isinstance(proxy,m3p.M3RtProxy)
    ## Returns the zlift object, None if none or more is found
    zlift = None
    zlift_names = proxy.get_available_components('m3joint_zlift')
    if len(zlift_names) > 1:
        print 'More than 1 zlift found, something is wrong.'
    elif len(zlift_names) == 1:
        try: 
            print 'Zlift names :', zlift_names
            zlift = m3z.M3JointZLift(zlift_names[0])
            proxy.subscribe_status(zlift)
        except Exception,e:
            print 'Zlift not found : ',e
            zlift = None
    return zlift

def get_omnibase(proxy):
    assert isinstance(proxy,m3p.M3RtProxy)
    ## Returns the omnibase object, None if none or more is found
    omni = None
    base_names = proxy.get_available_components('m3omnibase')
    if len(base_names) > 1:
        print 'More than 1 omnibase found, something is wrong.'
    elif len(base_names) == 1:
        try:
            print 'Base names :',base_names
            omni = m3o.M3OmniBase(base_names[0])
            proxy.subscribe_status(omni)
        except Exception,e:
            print 'Omni not found : ',e
            omni = None
    return omni

def get_bot(proxy):
    assert isinstance(proxy,m3p.M3RtProxy)
    ## Returns the bot object, None if none or more is found
    bot = None
    try:
        bot_name = m3t.get_robot_name()
        print 'bot name : ', bot_name
        bot = m3h.M3Humanoid(bot_name)  
        proxy.subscribe_status(bot)
    except Exception,e:
        print 'Something went wrong when trying to get the bot : ',e
        bot = None
    return bot

def get_right_hand(proxy):
    assert isinstance(proxy,m3p.M3RtProxy)
    ## Returns the right hand object, None if none or more is found
    right_hand = None
    try:
        right_hand_name = m3t.get_right_hand_name()
        if right_hand_name:
            print 'Right hand name : ', right_hand_name
            right_hand = m3.hand.M3Hand(right_hand_name)   
            proxy.subscribe_status(right_hand)
    except Exception,e:
        print 'No right hand found : ',e
        right_hand = None
    return right_hand

def get_left_hand(proxy):
    assert isinstance(proxy,m3p.M3RtProxy)
    ## Returns the left hand object, None if none or more is found
    left_hand = None
    try:
        left_hand_name = m3t.get_left_hand_name()
        if left_hand_name:
            print 'left_hand name : ', left_hand_name
            left_hand = m3.hand.M3Hand(left_hand_name)   
            proxy.subscribe_status(left_hand)
    except Exception,e:
        print 'No left hand found : ',e
        left_hand = None
    return left_hand
    
if __name__ == '__main__':
    server_started = False
    while not server_started:
        try:
            proxy = m3p.M3RtProxy()
            proxy.start()
            server_started= True
            
            right_arm = None
            left_arm  = None
            head    = None
            zlift       = get_zlift(proxy)
            omni        = get_omnibase(proxy)
            bot         = get_bot(proxy)
        
            right_hand  = get_right_hand(proxy)
            left_hand   = get_left_hand(proxy)
            if bot:
                all_chains = bot.get_available_chains()
                right_arm   = 'right_arm' in all_chains
                left_arm   = 'left_arm' in all_chains
                head = 'head' in all_chains
                
            print '*************** Available components ***************'
            print 'Zlift : ',zlift
            print 'Omni : ',omni
            print 'Bot : ',bot.get_available_chains()
            print 'Right hand : ',right_hand
            print 'Left hand : ',left_hand
            print '****************************************************'
            
            # Calibrate ZLift
        #    if zlift is not None:
        #        time.sleep(0.5)
        #        proxy.step()
        #        zlift.calibrate(proxy)
        #        time.sleep(0.5)
        #        proxy.step()
            
            # Calibrate Base
        #    if omni is not None:
        #        time.sleep(0.5)
        #        proxy.step()
        #        omni.calibrate(proxy)
        #        time.sleep(0.5)
            proxy.step()
            joints = []
            print 'Available Components'
            if omni:
            	print '- Omnibase'
                omni.set_local_position(0,0,0,proxy)
                omni.set_global_position(0,0,0,proxy)
                joints.append('X')    
                joints.append('Y')   
                joints.append('yaw')
            if zlift:
            	print '- Zlift'
                calib_zlift = zlift.get_encoder_calibrated()
                joints.append('zlift_joint')
            if right_arm:
            	print '- Right Arm'
                joints.append('right_arm_j0')
                joints.append('right_arm_j1')
                joints.append('right_arm_j2')
                joints.append('right_arm_j3')
                joints.append('right_arm_j4')
                joints.append('right_arm_j5')
                joints.append('right_arm_j6')
            if left_arm:
            	print '- left Arm'
                joints.append('left_arm_j0')
                joints.append('left_arm_j1')
                joints.append('left_arm_j2')
                joints.append('left_arm_j3')
                joints.append('left_arm_j4')
                joints.append('left_arm_j5')
                joints.append('left_arm_j6')
            if right_hand: 
            	print '- Right Hand'
                ndof_finger = 3
                flex_factor_index = [0.3] * (ndof_finger+1)
                flex_factor_ring = [0.3] * (ndof_finger+1)
                flex_factor_pinky = [0.3] * (ndof_finger+1)
                flex_factor_thumb = [0.3] * (ndof_finger)
                joints.append('right_hand_j0')
                joints.append('right_hand_j1')
                joints.append('right_hand_j2')
                joints.append('right_hand_j3')
                joints.append('right_hand_j4')
                joints.append('right_hand_j5')
                joints.append('right_hand_j6')
                joints.append('right_hand_j7')
                joints.append('right_hand_j8')
                joints.append('right_hand_j9')
                joints.append('right_hand_j10')
                joints.append('right_hand_j11')
                joints.append('right_hand_j12')
                joints.append('right_hand_j13')
                joints.append('right_hand_j14')
                joints.append('right_hand_j15')
            if left_hand:
            	print '- Left Hand'
                ndof_finger = 3
                flex_factor_index = [0.3] * (ndof_finger+1)
                flex_factor_ring = [0.3] * (ndof_finger+1)
                flex_factor_pinky = [0.3] * (ndof_finger+1)
                flex_factor_thumb = [0.3] * (ndof_finger)
                joints.append('left_hand_j0')
                joints.append('left_hand_j1')
                joints.append('left_hand_j2')
                joints.append('left_hand_j3')
                joints.append('left_hand_j4')
                joints.append('left_hand_j5')
                joints.append('left_hand_j6')
                joints.append('left_hand_j7')
                joints.append('left_hand_j8')
                joints.append('left_hand_j9')
                joints.append('left_hand_j10')
                joints.append('left_hand_j11')
                joints.append('left_hand_j12')
                joints.append('left_hand_j13')
                joints.append('left_hand_j14')
                joints.append('left_hand_j15')
            if head:
            	print '- Head'
                joints.append('head_j0')
                joints.append('head_j1')
                joints.append('head_j2')
                joints.append('head_j3')
                joints.append('head_j4')
                joints.append('head_j5')
                joints.append('head_j6')
                joints.append('head_j7_rt_eyelid_top')
                joints.append('head_j7_rt_eyelid_bottom')
                joints.append('head_j7_lt_eyelid_top')
                joints.append('head_j7_lt_eyelid_bottom')
               
            rospy.init_node("m3_joint_state_publisher")
            pub = rospy.Publisher("/joint_states", JointState)
            loop_rate = rospy.Rate(50.0)
            header = Header(0, rospy.Time.now(), '0')
            positions = [0.0]*len(joints)
            print 'Entering ROS Node'
            while not rospy.is_shutdown() and server_started:
                try:
                    header = Header(0, rospy.Time.now(), '0')
                    # Omnibase state
                    proxy.step()
                    i=0
                    if omni:
                        #omni_torque = omni.get_steer_torques()
                        omni_pos = omni.get_local_position()
                        omni_x = omni_pos[0]
                        omni_y = omni_pos[1]
                        omni_yaw = math.radians(omni_pos[2])
                        positions[i]=(omni_x); i=i+1
                        positions[i]=(omni_y); i=i+1
                        positions[i]=(omni_yaw); i=i+1
                    if zlift:
                        if calib_zlift:
                            zlift_z = zlift.get_pos_m()
                        else:
                            zlift_z = .5
                        positions[i]=(zlift_z-(0.32)); i=i+1#sol->haut_base + haut_base->capteur(repère 0.0)
                    if right_arm:
                        # Arm joint states
                        right_arm_th_rad = bot.get_theta_rad('right_arm')
                        for j in xrange(0,bot.get_num_dof('right_arm')):
                            positions[i]=(right_arm_th_rad[j]); i=i+1
                    if left_arm:
                        left_arm_th_rad = bot.get_theta_rad('left_arm')
                        for j in xrange(0,bot.get_num_dof('left_arm')):
                            positions[i]=(left_arm_th_rad[j]); i=i+1
                    if right_hand:
                        # Hand joint states
                        th = right_hand.get_theta_rad()
                        #Thumb
                        positions[i]=-th[0]+1.57 ; i=i+1
                        positions[i]=th[1] * flex_factor_thumb[0] ; i=i+1
                        positions[i]=th[1] * flex_factor_thumb[1] ; i=i+1
                        positions[i]=th[1] * flex_factor_thumb[2] ; i=i+1
                        #Index
                        positions[i]=th[2] * flex_factor_index[0] ; i=i+1
                        positions[i]=th[2] * flex_factor_index[1] ; i=i+1
                        positions[i]=th[2] * flex_factor_index[2] ; i=i+1
                        positions[i]=th[2] * flex_factor_index[3] ; i=i+1
                        #Ring
                        positions[i]=th[3] * flex_factor_ring[0] ; i=i+1
                        positions[i]=th[3] * flex_factor_ring[1] ; i=i+1
                        positions[i]=th[3] * flex_factor_ring[2] ; i=i+1
                        positions[i]=th[3] * flex_factor_ring[3] ; i=i+1
                        #Pinkie
                        positions[i]=th[4] * flex_factor_pinky[0] ; i=i+1
                        positions[i]=th[4] * flex_factor_pinky[1] ; i=i+1
                        positions[i]=th[4] * flex_factor_pinky[2] ; i=i+1
                        positions[i]=th[4] * flex_factor_pinky[3] ; i=i+1
                    if left_hand:
                        # Hand joint states
                        th = left_hand.get_theta_rad()
                        #Thumb
                        positions[i]=-th[0]+1.57 ; i=i+1
                        positions[i]=th[1] * flex_factor_thumb[0] ; i=i+1
                        positions[i]=th[1] * flex_factor_thumb[1] ; i=i+1
                        positions[i]=th[1] * flex_factor_thumb[2] ; i=i+1
                        #Index
                        positions[i]=th[2] * flex_factor_index[0] ; i=i+1
                        positions[i]=th[2] * flex_factor_index[1] ; i=i+1
                        positions[i]=th[2] * flex_factor_index[2] ; i=i+1
                        positions[i]=th[2] * flex_factor_index[3] ; i=i+1
                        #Ring
                        positions[i]=th[3] * flex_factor_ring[0] ; i=i+1
                        positions[i]=th[3] * flex_factor_ring[1] ; i=i+1
                        positions[i]=th[3] * flex_factor_ring[2] ; i=i+1
                        positions[i]=th[3] * flex_factor_ring[3] ; i=i+1
                        #Pinkie
                        positions[i]=th[4] * flex_factor_pinky[0] ; i=i+1
                        positions[i]=th[4] * flex_factor_pinky[1] ; i=i+1
                        positions[i]=th[4] * flex_factor_pinky[2] ; i=i+1
                        positions[i]=th[4] * flex_factor_pinky[3] ; i=i+1
                    if head:
		                  # Head state
		                  all_head_joints = bot.get_theta_rad('head')
		                  for j in xrange(0,len(all_head_joints)-1):
		                  	positions[i]=all_head_joints[j] ; i=i+1
		                  if len(all_head_joints)>0:
		                  	eye_lids_angle_rad = all_head_joints[-1]-m3t.deg2rad(35.0)
		                  	for j in xrange(4):
		                  		positions[i]=eye_lids_angle_rad ; i=i+1
                    
                    pub.publish(JointState(header, joints, positions, [], []))
                    loop_rate.sleep()
                except Exception,e:
                    if isinstance(e,rospy.ROSInterruptException):
                        proxy.step()
                        proxy.stop()
                        exit()
                    else:
                    	print 'Catching exception :',e
                        print 'M3rt serveur seems to be down, waiting for it to reboot : ',e
                        server_started= False
        except Exception,e:
            if isinstance(e,KeyboardInterrupt):
                print 'Exiting'
                exit()
            print 'Waiting for the M3 server to be launched'
            time.sleep(1.0)





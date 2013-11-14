#! /usr/bin/python
# -*- coding: utf-8 -*-
#Antoine Hoarau
#antoine.hoarau@inria.fr
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
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import m3.rt_proxy as m3p
import m3.humanoid as m3h
import m3.omnibase as m3o
import m3.toolbox as m3t
import m3.joint_zlift as m3z

import m3.component_factory as m3f
import time
import math
from m3.toolbox_core import M3Exception
if __name__ == '__main__':
    is_done = False
    while not is_done:
        try:
            server_started = False
            while not server_started:
                time.sleep(1.0)
                proxy = m3p.M3RtProxy()
                proxy.start()
                server_started=True
                
            #proxy.make_operational_all()#unecessary
            proxy.make_operational_all_shm()
            zlift = None
            zlift_names = proxy.get_available_components('m3joint_zlift')
            if len(zlift_names) != 1:
                print 'Zlift not found. Proceeding...'
            else:
                print 'Zlift names : ', zlift_names
                zlift = m3z.M3JointZLift(zlift_names[0])
                proxy.subscribe_status(zlift)
        
            omni = None
            base_name = proxy.get_available_components('m3omnibase')
            print 'Base names : ', base_name
        
            if len(base_name) != 1:
                    print 'Omnibase not found. Exiting'
                    exit()
            
            omni = m3o.M3OmniBase(base_name[0])
            proxy.subscribe_status(omni)
            if omni == None and zlift == None:
                exit()
                
            bot_name = m3t.get_robot_name()
            print 'bot name : ', bot_name
            bot = m3h.M3Humanoid(bot_name)   
            proxy.subscribe_status(bot)
        
            hands = proxy.get_available_components('m3hand')    
            print 'Hands available : ', hands
            if len(hands) > 0:
                hand_name = hands[0]
                print 'Using first hand : ', hand_name
            else:
                print 'No hands found'
            hand=m3.hand.M3Hand(hand_name)
            proxy.subscribe_status(hand)

            print 'M3Hand ndof : ', hand.ndof
            print 'M3Arm ndof : ', bot.get_num_dof('right_arm')
            print 'M3Head ndof : ', bot.get_num_dof('head')

            proxy.step()
            th_head = bot.get_theta_rad("head")
            th_arm = bot.get_theta_rad("right_arm")
            if len(th_arm)==0:
                print 'No arm theta,exiting'
                exit()
            if len(th_head)==0:
                print 'No head theta,exiting'
                exit()  
            ndof_finger = 3
            
            flex_factor_index = [0.3] * (ndof_finger+1)
            flex_factor_ring = [0.3] * (ndof_finger+1)
            flex_factor_pinky = [0.3] * (ndof_finger+1)
            flex_factor_thumb = [0.3] * (ndof_finger)
            joints = []
            omni.set_local_position(0,0,0,proxy)
            omni.set_global_position(0,0,0,proxy)
            joints.append('X')    
            joints.append('Y')   
            joints.append('yaw')    
            joints.append('zlift_joint')    
            
            joints.append('right_arm_j0')
            joints.append('right_arm_j1')
            joints.append('right_arm_j2')
            joints.append('right_arm_j3')
            joints.append('right_arm_j4')
            joints.append('right_arm_j5')
            joints.append('right_arm_j6')
            
            
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
            positions = [0.0]*len(joints)
            print 'Entering ROS Node'
            while not rospy.is_shutdown():
                header = Header(0, rospy.Time.now(), '0')
                # Omnibase state
                proxy.step()
                i=0
                #omni_torque = omni.get_steer_torques()
                omni_pos = omni.get_local_position()
                omni_x = omni_pos[0]
                omni_y = omni_pos[1]
                omni_yaw = math.radians(omni_pos[2])
                zlift_z = zlift.get_pos_m()
                
                positions[i]=omni_x ; i=i+1
                positions[i]=omni_y ; i=i+1
                positions[i]=omni_yaw ; i=i+1
                positions[i]=zlift_z-(0.32) ; i=i+1#sol->haut_base + haut_base->capteur(repère 0.0)
                # Arm joint states
                all_arm_joints = bot.get_theta_rad('right_arm')
                for j in xrange(0,bot.get_num_dof('right_arm')):
                    positions[i]=all_arm_joints[j] ; i=i+1
                # Hand joint states
                th = hand.get_theta_rad()
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
                # Head state
                all_head_joints = bot.get_theta_rad('head')
                for j in xrange(0,len(all_head_joints)-1):
                    positions[i]=all_head_joints[j] ; i=i+1
                if len(all_head_joints)>0:
                    eye_lids_angle_rad = all_head_joints[-1]-m3t.deg2rad(35.0)
                    for j in xrange(4):
                        positions[i]=eye_lids_angle_rad ; i=i+1
    
                pub.publish(JointState(header, joints, positions,[],[] ))#[0] * len(positions), [0] * len(positions)))
                loop_rate.sleep()
            if rospy.is_shutdown():
                print 'Stop requested'
                is_done = True
        except Exception,e:
            print e
            print "Restarting"
    print 'Exiting'
    proxy.stop()
    print 'Exit'
    exit()





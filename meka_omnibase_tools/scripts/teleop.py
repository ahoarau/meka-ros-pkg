#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg   import Joy

class Teleop:
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.sub_joy     = rospy.Subscriber("joy", Joy, self.joy_cb)
        self.sub_aux     = rospy.Subscriber("~aux_cmd_vel", Twist, self.aux_cb)
        
        self.lin_scaler = 0.10
        self.ang_scaler = 0.20
        self.cmd_vel    = Twist()
        self.aux_vel    = Twist()
        self.last_cmd   = rospy.Time()
        self.last_aux   = rospy.Time()

        self.teleop_active = False

    def joy_cb(self, msg):
        self.vel = Twist()
        if msg.buttons[4] == 0:
            self.teleop_active = False
            return

        self.teleop_active = True

        self.cmd_vel.linear.x  = self.lin_scaler * msg.axes[1]
        self.cmd_vel.linear.y  = self.lin_scaler * msg.axes[0]
        self.cmd_vel.angular.z = self.ang_scaler * msg.axes[3]

        self.last_cmd = rospy.Time.now()

    def aux_cb(self, msg):
        self.aux_vel  = msg
        self.last_aux = rospy.Time.now()

    def publish(self):
        elapsed_cmd = rospy.Time.now() - self.last_cmd
        elapsed_aux = rospy.Time.now() - self.last_aux
        
        if   (self.teleop_active):
            self.pub_cmd_vel.publish(self.cmd_vel)
        elif (elapsed_aux.to_sec() < 0.2):
            self.pub_cmd_vel.publish(self.aux_vel)
        #else:
        #    self.pub_cmd_vel.publish(Twist()) # Zero command

if __name__ == "__main__":
    rospy.init_node("meka_teleop")
    node = Teleop()

    while (not rospy.is_shutdown()):
        node.publish()
        rospy.sleep(0.1)


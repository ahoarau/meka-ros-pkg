#!/usr/bin/env python

# Takes care of sending Twist command to the omnibase and publishing the
# odometry both as a topic and a TF transform.

import m3.rt_proxy as m3p
import m3.meka_omnibase_control as m3o
import m3.component_factory     as m3f
import rospy
import tf
import math
import numpy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg      import Odometry

class OmniBridge:
    def __init__(self):
        # M3-specific stuff
        self.omni = m3o.MekaOmnibaseControl('meka_omnibase_control_mb2', 'meka_omnibase_control')

        self.proxy = m3p.M3RtProxy()
        self.proxy.start()
        self.proxy.make_operational_all()
        self.proxy.subscribe_status(self.omni)
        self.proxy.publish_command(self.omni)

        # TODO: Use the method associated with omnibase instead.
        pwr_name = self.proxy.get_available_components('m3pwr')
        self.pwr = m3f.create_component(pwr_name[1])
        self.proxy.subscribe_status(self.pwr)
        self.proxy.publish_command(self.pwr)
        self.pwr.set_motor_power_on()

        self.proxy.step()

        # ROS stuff
        self.sub_cmd_vel = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb, queue_size=1)
        self.pub_odom    = rospy.Publisher("odom", Odometry, queue_size=1)
        self.tf_bc       = tf.TransformBroadcaster()

        self.max_lin  = rospy.get_param("~max_lin_vel", 0.30)
        self.max_ang  = rospy.get_param("~max_ang_vel", 1.00)
        self.cmd_vel  = Twist()
        self.last_cmd = rospy.Time(0)
        self.timeout  = rospy.Duration(0.25)

        self.diag_i = 0

    def __del__(self):
        self.stop()

    def stop(self):
        print "Stopping proxy..."
        self.proxy.stop()
        print "Done."

    def cmd_vel_cb(self, msg):
        self.cmd_vel  = msg
        self.last_cmd = rospy.Time.now() 

    def step(self):
        now      = rospy.Time.now()
        elapsed  = now - self.last_cmd


        # Make the transform available in the 'future' to avoid extrapolation errors
        # Same thing AMCL and robot_state_publisher for static transform do.
        odom_time = now + rospy.Duration(0.50)

        #rospy.logdebug("Time elapsed since last command: " + str(elapsed.to_sec()))
        if (elapsed > self.timeout):
            # Too much time passed since last command, zero it.
            self.cmd_vel = Twist()

        xd = self.cmd_vel.linear.x
        yd = self.cmd_vel.linear.y
        td = self.cmd_vel.angular.z
        
        xda = math.fabs(self.cmd_vel.linear.x)
        yda = math.fabs(self.cmd_vel.linear.y)
        tda = math.fabs(self.cmd_vel.angular.z)

        # Saturate according to velocity limits:
        if (xda > self.max_lin):
            xds = self.cmd_vel.linear.x / xda
            xd = xds * self.max_lin
        if (yda > self.max_lin):
            yds = self.cmd_vel.linear.y / yda
            yd = yds * self.max_lin
        if (tda > self.max_ang):
            tds = self.cmd_vel.angular.z / tda
            td = tds * self.max_ang


        self.omni.set_mode_on()
        self.omni.set_desired_twist(xd,yd,td)
        for i in range(0,4):
            self.omni.set_caster_tqr(i, 1.0)
        self.proxy.step()

        # Odometry
        (x, y, t)       = self.omni.get_global_position()
        (oxd, oyd, otd) = self.omni.get_current_twist()

        q = tf.transformations.quaternion_from_euler(0, 0, t)

        self.tf_bc.sendTransform((x, y, 0),
                                 q,
                                 odom_time,
                                 "base_footprint",
                                 "odom")

        odom = Odometry()
        odom.header.frame_id       = "odom"
        odom.header.stamp          = odom_time
        odom.child_frame_id        = "base_footprint"
        odom.pose.pose.position.x  = x
        odom.pose.pose.position.y  = y
        odom.pose.pose.position.z  = 0
        odom.pose.pose.orientation = Quaternion(*q)

        odom.twist.twist.linear.x  = oxd
        odom.twist.twist.linear.y  = oyd
        odom.twist.twist.angular.z = otd

        self.pub_odom.publish(odom)

        # Diagnostics
        self.diag_i = self.diag_i + 1
        if (self.diag_i > 5):
            rospy.logdebug("alpha: " + str(self.omni.status.alpha))
            rospy.logdebug("beta:  " + str(self.omni.status.beta))
            rospy.logdebug("betad: " + str(self.omni.status.beta_d))
            rospy.logdebug("phid:  " + str(self.omni.status.phi_d))
            self.diag_i = 0

            for i in range(0,4):
                l = self.omni.status.l[i]
                alpha = self.omni.status.alpha[i]
                x = l * math.cos(alpha)
                y = l * math.sin(alpha)
                q = tf.transformations.quaternion_from_euler(
                        0,
                        0,
                        alpha + self.omni.status.beta[i] - math.pi/2.0)
                self.tf_bc.sendTransform((x,y,0),
                                         q,
                                         odom_time,
                                         "caster_" + str(i),
                                         "base_link")
                # NOTE: This is a huge limitation in rospy's TF node: if the 
                # transform did not have time to be sent, it gets overwritten (?)
                rospy.sleep(0.01)

if __name__=='__main__':

    rospy.init_node('omni_bridge')

    ot = OmniBridge()

    loop_rate = rospy.Rate(50)
    while (not rospy.is_shutdown()):
        ot.step()
        loop_rate.sleep()

    ot.stop()
        


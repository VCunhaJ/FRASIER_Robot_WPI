#!/usr/bin/env python
import math

import rospy

from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from roboteq_mc_nxtgen_driver.msg import RPM
from roboteq_mc_nxtgen_driver.srv import *
    
if __name__ == '__main__':
    rospy.init_node('roboteq_mc_nxtgen_driver_tests')
    
    test = rospy.get_param('~test', "open_loop_no_knmtx")
    
    rospy.loginfo("Wait until the the motor controller driver starts")
    rospy.wait_for_service('/roboteq_mc_nxtgen_driver/custom_cmd')
    rospy.loginfo("Start testing...")
    
    if test == "open_loop_no_knmtx":
        rospy.loginfo("Test OPEN LOOP WITHOUT KINEMATICS")
        pub = rospy.Publisher("/cmd_power", RPM)
        u = 0.2
        for v in [[u,u], [-u,-u], [u,-u], [-u,u]]:
            rospy.loginfo("SET [wheel1, wheel2] = [%f, %f]" % (v[0],v[1]))
            cmd = RPM(v)
            for k in range(0,10):
                pub.publish(cmd)
                rospy.sleep(0.3)
            rospy.sleep(3)
    elif test == "closed_loop_no_knmtx":
        rospy.loginfo("Test CLOSED LOOP WITHOUT KINEMATICS")
        pub = rospy.Publisher("/cmd_rpm", RPM)
    
        u = 60
        for v in [[u,u], [-u,-u], [u,-u], [-u,u]]:
            rospy.loginfo("SET [wheel1, wheel2] = [%f rpm, %f rpm]" % (v[0],v[1]))
            cmd = RPM(v)
            for k in range(0,10):
                pub.publish(cmd)
                rospy.sleep(0.3)
            rospy.sleep(3)
    elif test == "open_loop_with_knmtx":
        rospy.loginfo("Test OPEN LOOP WITH KINEMATICS")
        pub = rospy.Publisher("/cmd_vel", Twist)
        v = 0.3
        w = 0.6
        for vels in [[v,0], [-v,0], [0,w], [0,-w]]:
            rospy.loginfo("SET [V, omega] = [%f m/s, %f rad/s]" % (vels[0],vels[1]))
            cmd = Twist()
            cmd.linear.x = vels[0]
            cmd.angular.z = vels[1]
            for k in range(0,10):
                pub.publish(cmd)
                rospy.sleep(0.3)
            rospy.sleep(3)        
    elif test == "closed_loop_with_knmtx":
        rospy.loginfo("Test CLOSED LOOP WITH KINEMATICS")
        pub = rospy.Publisher("/cmd_vel", Twist)
        v = 0.3
        w = 0.6
        for vels in [[v,0], [-v,0], [0,w], [0,-w]]:
            rospy.loginfo("SET [V, omega] = [%f m/s, %f rad/s]" % (vels[0],vels[1]))
            cmd = Twist()
            cmd.linear.x = vels[0]
            cmd.angular.z = vels[1]
            for k in range(0,10):
                pub.publish(cmd)
                rospy.sleep(0.3)
            rospy.sleep(3)
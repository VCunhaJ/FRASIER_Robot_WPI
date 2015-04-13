#!/usr/bin/env python
from threading import Thread
import math

import rospy, tf

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from roboteq_mc_nxtgen_driver.msg import RPM
from roboteq_mc_nxtgen_driver.srv import *

from RoboteqNXTMotorController import *

def pubBattery(mc, topic, batPubPeriod):
    pub = rospy.Publisher(topic, Float32)
    while(not rospy.is_shutdown()):
        voltages = None
        try:
            voltages = mc.getMCVoltages()
        except IOError:
            rospy.signal_shutdown("Failed to read battery voltage from the "
                "motor controller after multiple attempts. Details: %s" % str(e))
            continue                
        pub.publish(Float32(voltages[1]))
        rospy.sleep(batPubPeriod)
    
def pubRpm(mc, topic, rpmPubPeriod):
    pub = rospy.Publisher(topic, RPM)    
    while(not rospy.is_shutdown()):
        rpms = None
        try:
            rpms = mc.getRpms()
        except IOError as e:
            rospy.signal_shutdown("Failed to read RPMs from the "
                "motor controller after multiple attempts. Details: %s" % str(e))
            continue                
        pub.publish(RPM(rpms))
        rospy.sleep(rpmPubPeriod)
    
def pubOdom(mc, topic, period, baseFrame, odomFrame):
    lastRun = rospy.get_time()
    x = 0.0
    y = 0.0
    phi = 0.0
    pub = rospy.Publisher(topic, Odometry)
    tfbr = tf.TransformBroadcaster()
    
    while(not rospy.is_shutdown()):
        v = None
        try:
            v = mc.getVelocities()
        except IOError as e:
            rospy.signal_shutdown("Failed to read velocities from the "
                "motor controller after multiple attempts. Details: %s" % str(e))
            continue
                
        #integrate pose
        curTime = rospy.get_time()
        dt = curTime - lastRun
        lastRun = curTime
        a1 = v[0]
        a2 = math.cos(phi)
        a3 = dt
        x += a1*a2*a3#v[0] * math.cos(phi) * dt
        y += v[0] * math.sin(phi) * dt
        phi += v[1] * dt
        
        #Send odometry msg
        msg = Odometry()
        msg.header.frame_id = odomFrame
        msg.child_frame_id = baseFrame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        quatrn = tf.transformations.quaternion_from_euler(0, 0, phi)
        msg.pose.pose.orientation = Quaternion(*quatrn)
        msg.twist.twist.linear.x = v[0]
        msg.twist.twist.angular.z = v[1]
        pub.publish(msg)
        
        #Broadcast transform
        tfbr.sendTransform((x, y, 0.0), quatrn, rospy.Time.now(),
            baseFrame, odomFrame)
        rospy.sleep(period)

    
if __name__ == '__main__':
    rospy.init_node('roboteq_mc_nxtgen_driver')
    
    #Motor controller parameters
    mcPrms = {
        'port'              : rospy.get_param('~port', "/dev/ttyACM0"),
        'timeout'           : rospy.get_param('~timeout', 0.04),
        'sendAttempts'      : rospy.get_param('~sendAttempts', 100),
        'onConnect'         : rospy.get_param('~onConnect', []),
        'portCfg'           : rospy.get_param('~portCfg', {}),
        'debug'             : lambda msg: rospy.logdebug("\tMC: "+msg)
    }

    #Forward and inverse kinematics matricies
    fwdKnmtx = rospy.get_param('~fwdKnmtx', None)
    if fwdKnmtx:
        mcPrms.update({
            'fwdKnmtx': fwdKnmtx,
            'invKnmtx': rospy.get_param('~invKnmtx', None)
        })

    closedLoop      = rospy.get_param('~closedLoop', False)
    
    #Publshers publish period
    rpmPubPeriod        = rospy.get_param('~rpmPubPeriod', 0.1)
    odomPubPeriod       = rospy.get_param('~odomPubPeriod', 0.1)
    batteryPubPeriod    = rospy.get_param('~batteryPubPeriod', 1.0)    
    baseFrame           = rospy.get_param('~baseFrame', "base_link")
    odomFrame           = rospy.get_param('~odomFrame', "odom")

    mc = None
    if not fwdKnmtx:
        if closedLoop:
            mc = MotorControllerClosedLoop(**mcPrms)
            rospy.loginfo("Motor controller is in CLOSED LOOP mode " + 
                "WITHOUT KINEMATICS")
            rospy.Subscriber("/cmd_rpm", RPM, 
                lambda msg: mc.setRpms(msg.data))                
        else:
            mc = MotorControllerOpenLoop(**mcPrms)
            rospy.loginfo("Motor controller is in OPEN LOOP mode " + 
                "WITHOUT KINEMATICS")
            rospy.Subscriber("/cmd_power", RPM, 
                lambda msg: mc.setDutyCycles(msg.data))                     
        Thread(target=pubRpm, args=[mc, "/rpm", rpmPubPeriod]).start()
    else:
        if closedLoop:
            mc = MotorControllerClosedLoopWithKnmtx(**mcPrms)
            rospy.loginfo("Motor controller is in CLOSED LOOP mode " + 
                "WITH KINEMATICS")
        else:
            mc = MotorControllerOpenLoopWithKnmtx(**mcPrms)
            rospy.loginfo("Motor controller is in OPEN LOOP mode " + 
                "WITH KINEMATICS")
        rospy.Subscriber("/cmd_vel", Twist, 
            lambda msg: mc.setVelocities([msg.linear.x, msg.angular.z]))
        Thread(target=pubOdom, args=[mc, "/odom", odomPubPeriod,
                                            baseFrame, odomFrame]).start()
     
    rospy.Service("~custom_cmd", CustomCmd, 
        lambda req: CustomCmdResponse(mc.sendMessageReliably(req.req)))
        
    pubBattery(mc, "/battery", batteryPubPeriod)



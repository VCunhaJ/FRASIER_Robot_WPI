#!/usr/bin/python

import roslib
import rospy
import serial
import string

from geometry_msgs.msg import PoseStamped
from time import time, sleep
from std_msgs.msg import String

#roslib.load_manifest('odom_publisher')

pub = rospy.Publisher('/pixy', PoseStamped)
rospy.init_node('pixy', anonymous=True)

default_port1="/dev/ttyACM1"
port1 = rospy.get_param('device', default_port1)
# Check your COM port and baud rate
ser1 = serial.Serial(port=port1,baudrate=9600, timeout=1)
numsig1blocks = 0
numsig2blocks = 0
xval = yval = distance = 0.0
sig1blocks = [0]*4
sig2blocks = [0]*4
sig1 = []
sig2 = []
#sig1 = [[] for i in xrange(4)]
#sig2 = [[] for i in xrange(4)]
numblocks = 0
sequence = 0
rospy.sleep(2)

while 1:
    line1 = ser1.readline()
    try:
        blocklist = [int(x) for x in line1.split(",")]
        numblocks = blocklist[0]
        print "Number of Blocks Detected =",numblocks
        for x in range(0,numblocks):
            if blocklist[(5*x)+1] == 1:
                sig1blocks[0] = blocklist[(5*x)+2]
                sig1blocks[1] = blocklist[(5*x)+3]
                sig1blocks[2] = blocklist[(5*x)+4]
                sig1blocks[3] = blocklist[(5*x)+5]
                sig1.append(sig1blocks)
                numsig1blocks += 1
            if blocklist[5*x+1] == 2:
                sig2blocks[0] = blocklist[(5*x)+2]
                sig2blocks[1] = blocklist[(5*x)+3]
                sig2blocks[2] = blocklist[(5*x)+4]
                sig2blocks[3] = blocklist[(5*x)+5]
                sig2.append(sig2blocks)
                numsig2blocks += 1
    except:
        pass


    if numsig1blocks>0 and numsig2blocks>0:
        if numsig1blocks == 1 and numsig2blocks == 1:
            xval = (sig1[0][0] + sig2[0][0])/2
            yval = (sig1[0][1] + sig2[0][1])/2
            distance = (((-0.00665 * (sig1[0][2]*sig1[0][3])) + (-0.00665 * (sig2[0][2]*sig2[0][3])))/2) + 48.825
        elif numsig1blocks == 1 and numsig2blocks > 1:
            #Compare sig1 with other sig2
            ymin = 1000
            for i in range(0,numsig2blocks):
                compare = abs(sig1[0][1] - sig2[i][1])
                if compare < ymin:
                    ymin = compare
                    blockcompare = i
            xval = (sig1[0][0] + sig2[blockcompare][0])/2
            yval = (sig1[0][1] + sig2[blockcompare][1])/2
            distance = (((-0.00665 * (sig1[0][2]*sig1[0][3])) + (-0.00665 * (sig2[blockcompare][2]*sig2[blockcompare][3])))/2) + 48.825
        elif numsig1blocks > 1 and numsig2blocks == 1:
            #compare sig2 with other sig1
            ymin = 1000
            for i in range(0,numsig1blocks):
                compare = abs(sig2[0][1] - sig1[i][1])
                if compare < ymin:
                    ymin = compare
                    blockcompare = i
            xval = (sig1[blockcompare][0] + sig2[0][0])/2
            yval = (sig1[blockcompare][1] + sig2[0][1])/2
            distance = (((-0.00665 * (sig1[blockcompare][2]*sig1[blockcompare][3])) + (-0.00665 * (sig2[0][2]*sig2[0][3])))/2) + 48.825
        else:
            #compare sig1 blocks with sig2 blocks
            ymin = 1000
            blockcompare1 = blockcompare2 = 0
            for i in range(0,numsig1blocks):
                for x in range(0,numsig2blocks):
                    compare = abs(sig1[i][1] - sig2[x][1])
                    if compare < ymin:
                        ymin = compare
                        blockcompare1 = i
                        blockcompare2 = x
            xval = (sig1[blockcompare1][0] + sig2[blockcompare2][0])/2
            yval = (sig1[blockcompare1][1] + sig2[blockcompare2][1])/2
            distance = (((-0.00665 * (sig1[blockcompare1][2]*sig1[blockcompare1][3])) + (-0.00665 * (sig2[blockcompare2][2]*sig2[blockcompare2][3])))/2) + 48.825
    elif numblocks == 1:
        if numsig1blocks == 1 and numsig2blocks == 0:
            xval = sig1[0][0]
            yval = sig1[0][1]
            distance = (-0.00665*(sig1[0][2]*sig1[0][3])) + 48.825
        elif numsig1blocks == 0 and numsig2blocks == 1:
            xval = sig2[0][0]
            yval = sig2[0][1]
            distance = (-0.00665*(sig2[0][2]*sig2[0][3])) + 48.825
    else:
        print "No Blocks Detected"

    distance *= 0.0254
    xval = (xval-160)*(distance/320)
    yval = (yval-100)*(distance/200)

    pixy = PoseStamped()
    pixy.header.stamp = rospy.Time.now()
    sequence+=1
    pixy.header.seq = sequence
    pixy.header.frame_id = '/pixy'
    #pixy.pose.pose.position = Point(x, y, z)
    #pixy.pose.pose.orientation = Quaternion(*(kdl.Rotation.RPY(R, P, Y).GetQuaternion()))


    pixy.pose.position.x = xval
    pixy.pose.position.y = yval
    pixy.pose.position.z = distance
    pos = (pixy.pose.position.x,pixy.pose.position.y,pixy.pose.position.z)

    pixy.pose.orientation.x = 0
    pixy.pose.orientation.y = 0
    pixy.pose.orientation.z = 0
    pixy.pose.orientation.w = 0

    sig1 = []
    sig2 = []
    numsig1blocks = 0
    numsig2blocks = 0
    ser1.close
    rospy.loginfo(pixy)
    pub.publish(pixy)

# camera 0,0 on top left corner from camera reference
# x goes from 0 to 319
# y goes from 0 to 199
# Distance x unit equals 2x units in x direction and x units in y direction
# Distance in inches = (-0.006644502*Area) + 48.825

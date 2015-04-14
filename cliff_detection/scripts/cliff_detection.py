#!/usr/bin/python

import roslib
import rospy

import serial
import string

from time import time, sleep
from std_msgs.msg import Int32

pub = rospy.Publisher('cliff_status', Int32)
rospy.init_node('cliff_detection', anonymous=True)

default_port1="/dev/ttyACM0"
port1 = rospy.get_param('device', default_port1)
# Check your COM port and baud rate
ser1 = serial.Serial(port=port1,baudrate=9600, timeout=1)

default_port2="/dev/ttyACM0"
port2 = rospy.get_param('device', default_port2)
ser2 = serial.Serial(port=port2,baudrate=9600, timeout=1)

IR1=IR2=IR3=IR4=US1=US2=US3=US4=set1=set2=set3=set4=total=0
cliff_status = 0
min_cliff_distance = 8  # Need a minimum of 8 inches of clearance from ground to be considered a cliff/edge
print "0: Safe\n1: Slowdown\n2: Arbitrary\n3: Stop"
rospy.sleep(3)

class switch(object):
    value = None
    def __new__(class_, value):
        class_.value = value
        return True

def case(*args):
    return any((arg == switch.value for arg in args))

while 1:
    line1 = ser1.readline()
    words1 = string.split(line1,",")    # Fields split
    if len(words1) > 2:
        try:
            IR1 = words1[0]
            IR2 = words1[1]
            IR3 = words1[2]
            IR4 = words1[3]
        except Exception as e:
            print e
    ser1.close

    line2 = ser2.readline()
    words2 = string.split(line2,",")    # Fields split

    if len(words2) > 2:
        try:
            US1 = words2[0]
            US2 = words2[1]
            US3 = words2[2]
            US4 = words2[3]
        except Exception as e:
            print e
    ser2.close

    if(IR1>min_cliff_distance or US1>min_cliff_distance):
        set1 = 1      # Right-Forward
    else:
        set1 = 0
    if(IR2>min_cliff_distance or US2>min_cliff_distance):
        set2 = 1      # Right - Rear
    else:
        set2 = 0
    if(IR3>min_cliff_distance or US3>min_cliff_distance):
        set3 = 1      # Left - Forward
    else:
        set3 = 0
    if(IR4>min_cliff_distance or US4>min_cliff_distance):
        set4 = 1      # Left - Rear
    else:
        set4 = 0

	total = (set1) + (2*set2) + (4*set3) + (8*set4) # Pseudo-Binary case state
	while switch(total):
		if case(0):
		    cliff_status = 0#"safe"
		    break
		if case(1, 4):
		    cliff_status = 1#"slowdown"  # Front sets triggered
		    break
		if case(2, 5, 6, 8, 9, 10):
		    cliff_status = 2#"arbitrary"
		if case(3, 7, 11, 12, 13, 14, 15):  # If Either side both sets are triggered
		    cliff_status = 3#"stop"
		    break
		break

    rospy.loginfo(cliff_status)
    pub.publish(cliff_status)

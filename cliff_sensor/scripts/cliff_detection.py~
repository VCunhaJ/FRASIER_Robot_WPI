#!/usr/bin/env python

import roslib; roslib.load_manifest('cliff_sensor')
import rospy

import serial
import string
import math

from time import time
from sensor_msgs.msg import Imu
from cliff_dection.msg import cliff_detection

rospy.init_node("node")
pub = rospy.Publisher('cliff_detection', cliff_detection)

cliff_detectionMsg = cliff_detection()

default_port='/dev/ttyUSB0'
port = rospy.get_param('device', default_port)
# Check your COM port and baud rate
ser = serial.Serial(port=port,baudrate=9600, timeout=1)

#f = open("Serial"+str(time())+".txt", 'w')

IR1=0
IR2=0
IR3=0
IR4=0

rospy.sleep(2)# Sleep for 8 seconds to wait for the board to boot then only write command.
count = 0
ser.write('#ox' + chr(13)) # To start display angle and sensor reading in text
while 1:
    line = ser.readline()
    #line = line.replace("#YPR=","")   # Delete "#YPR="
    #print line
    #f.write(line)                     # Write to the output log file
    words = string.split(line,",")    # Fields split
    if len(words) > 2:
       try:
         #print "normal"
         IR1 = float(words[0])
         IR2 = float(words[1])
         IR3 = float(words[2])
         IR4 = float(words[3])

        except Exception as e:
           print e
  
     cliff_detectionMsg.IR1 = IR1
     cliff_detectionMsg.IR2 = IR2
     cliff_detectionMsg.IR3 = IR3
     cliff_detectionMsg.IR4 = IR4
     pub.publish(cliff_detectionMsg)

ser.close
#f.close

#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"Cliff Status: %s",data.data)
    
def get_cliff_status():
    rospy.init_node('get_cliff_status', anonymous=True)
    rospy.Subscriber("cliff_status", String, callback)
    rospy.spin()
        
if __name__ == '__main__':
    get_cliff_status()

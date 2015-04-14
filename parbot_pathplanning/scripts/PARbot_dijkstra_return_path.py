#!/usr/bin/env python

#Olivia Hugal
#Jeffrey Orszulak
#Last Revised 1/29/2014

import roslib; roslib.load_manifest('parbot_pathplanning')
import rospy
from PARbot_dijkstra import *
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from parbot_pathplanning.srv import PARbotPathPlanning, PARbotPathPlanningResponse #import service

class PathFollower:
    def __init__(self, target):
        rospy.init_node("path_printer")
        self.target = target

        rospy.loginfo("Waiting for service")
        rospy.wait_for_service('PARbot_Path_Planning')
        self.PARbot_Path_Planning = rospy.ServiceProxy('PARbot_Path_Planning', PARbotPathPlanning)
        self.sub = rospy.Subscriber("map", OccupancyGrid, self.plan)
        rospy.loginfo("Started")

    def plan(self, grid):
        try:
            #Print to screen that service has received a grid
            rospy.loginfo("Received OccupancyGrid")
            "formulate request to service with start location defined and target from class definition"
            resp = self.PARbot_Path_Planning(grid = grid, start = Point(-2.6, -2,0), target = self.target)
            #Print Path to screen"
            if resp.pathExists:
                rospy.loginfo("Path: %s"%[(p.x, p.y) for p in resp.waypoints])
            else:
                print "no path exists, start or target not valid"
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s"%e)

if __name__ == "__main__":
    PathFollower(Point(4.8, 0.4, 0))
    rospy.spin() # keep running
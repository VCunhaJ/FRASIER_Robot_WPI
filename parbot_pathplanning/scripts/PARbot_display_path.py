#!/usr/bin/env python

#Olivia Hugal
#Jeffrey Orszulak
#Last Revised 2/24/2014

import roslib; roslib.load_manifest('parbot_pathplanning')
import rospy
import tf
from PARbot_dijkstra import *
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseArray, Pose
from parbot_pathplanning.srv import PARbotPathPlanning, PARbotPathPlanningResponse #import service
from math import pi, atan2, sqrt

#Defines
#FRAME = "/base_footprint"
FRAME = "/map"

class PathFollower:
    def __init__(self, target):
        rospy.init_node("path_printer")
        self.target = target
        self.rviz_path = []
        self.orientation = 0 #Starting orientation facing positive x-axis
        self.points = PoseArray()
        rospy.loginfo("Waiting for service")
        rospy.wait_for_service('PARbot_Path_Planning')
        self.PARbot_Path_Planning = rospy.ServiceProxy('PARbot_Path_Planning', PARbotPathPlanning)
        self.sub = rospy.Subscriber("map", OccupancyGrid, self.plan)
        rospy.loginfo("Started")
        self.rviz_path_pub = rospy.Publisher("rviz_path", PoseArray)

    def get_pose(self, x_in, y_in, theta):
        pose = Pose()
        pose.position.x = x_in
        pose.position.y = y_in
        pose.position.z = 0
        x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, theta)
        pose.orientation.x = x
        pose.orientation.y = y
        pose.orientation.z = z
        pose.orientation.w = w
        return pose

    def get_orientation(self, current, future):
        x = future.x - current.x
        y = future.y - current.y
        desired_orientation = atan2(y, x)
        #print "current, future, desired_orientation" , current.x, future.x, desired_orientation
        self.orientation = desired_orientation #assume robot rotates as desired
        return desired_orientation

    def plan(self, grid):
        try:
            #Print to screen that service has received a grid
            rospy.loginfo("Received OccupancyGrid")
            "formulate request to service with start location defined and target from class definition"
            resp = self.PARbot_Path_Planning(grid = grid, start = Point(0, 0,0), target = self.target)

            #Display path in RVIZ using a pose array
            rviz_waypoints = resp.waypoints[:] #copy of waypoints for getting orientation
            length = len(rviz_waypoints) #determine number of points for "for loop"
            for i in range(0, (length-1)):
                point = rviz_waypoints[i] #get current point
                self.rviz_path.append(self.get_pose(point.x, point.y, self.get_orientation(rviz_waypoints[i], rviz_waypoints[i+1])))
            self.points.header.frame_id = FRAME #set frame for pose array
            self.points.poses = self.rviz_path[:] #set poses for publisher
            for pose in self.points.poses:
                print pose
            self.rviz_path_pub.publish(self.points)

            #Print Path to screen
            if resp.pathExists:
                rospy.loginfo("Path: %s"%[(p.x, p.y) for p in resp.waypoints])
            else:
                print "no path exists, start or target not valid"
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s"%e)

if __name__ == "__main__":
    #keep printing path as fast as possible, can limit if needed
    while not rospy.is_shutdown():
        PathFollower(Point(1, 1, 0))
        rospy.spin() # keep running
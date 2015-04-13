#!/usr/bin/env python

#Olivia Hugal
#Jeffrey Orszulak
#Last Revised 2/5/2014

import roslib; roslib.load_manifest('parbot_pathplanning')
import rospy
from PARbot_dijkstra import *
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Twist, Vector3
from parbot_pathplanning.srv import PARbotPathPlanning, PARbotPathPlanningResponse #import service\
from math import pi, atan2, sqrt

class TurnCommand:
    def __init__(self, driver, orientation):
        self.driver = driver
        self.orientation = orientation

    def execute(self):
        start = rospy.get_rostime()
        turn_time = rospy.Duration.from_sec(abs(self.orientation/(pi/6)))
        if self.orientation > 0:
            turn_speed = pi/6
        elif self.orientation < 0:
            turn_speed = -pi/6
        while rospy.get_rostime() - start < turn_time:
            self.driver.drive(0, turn_speed)
            rospy.sleep(0.005)

class DriveCommand:
    def __init__(self, driver, distance):
        self.driver = driver
        self.distance = distance

    def execute(self):
        self.driver.drive(0.5, 0)
        rospy.sleep(self.distance/0.5)



class PathFollower:
    def __init__(self, target):
        rospy.init_node('path_printer')
        self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
        self.target = target
        self.commands = []
        self.path_to_drive = False

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
            resp = self.PARbot_Path_Planning(grid = grid, start = Point(0.6, 4,0), target = self.target)
            #Print Path to screen"
            if resp.pathExists:
                rospy.loginfo("Path: %s"%[(p.x, p.y) for p in resp.waypoints])
                self.commands = self.get_commands(resp.waypoints)
                print 'Commands: ' , self.commands
                self.path_to_drive = True
            else:
                print "no path exists, start or target not valid"
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s"%e)


    def get_commands(self, waypoints):
        commands = []
        #commands.append(TurnCommand(self, -10)) #padded with 1 dummy command
        orientation = 0 #Starting orientation facing positive x-axis
        while waypoints:
            print "ORIENTATION: " , orientation
            current = waypoints.pop(0)
            if not waypoints: break
            future = waypoints[0]
            x = future.x - current.x
            y = future.y - current.y
            desired_orientation = atan2(y, x)
            if orientation != desired_orientation:
                commands.append(TurnCommand(self, (desired_orientation - orientation)))
            commands.append(DriveCommand(self, sqrt(x**2 + y**2)))
            orientation = desired_orientation #assume robot rotates as desired
        return commands

    def drive(self, linear, angular):
        self.pub.publish(Twist(linear=Vector3(x=linear, y=0, z=0),
                               angular=Vector3(x=0, y=0, z=angular)))

    def run(self):
        while True:
            if self.path_to_drive:
                for command in self.commands:
                    command.execute()
                self.path_to_drive = False
            rospy.sleep(0.1)

if __name__ == "__main__":
    PathFollower(Point(0.8, 2.4, 0)).run()
    rospy.spin() # keep running
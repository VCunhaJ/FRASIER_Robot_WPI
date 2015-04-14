#!/usr/bin/env python

#Olivia Hugal
#Jeffrey Orszulak
#Last Revised 1/29/2014

import roslib; roslib.load_manifest('parbot_pathplanning')
import rospy
import math
from PARbot_dijkstra import *
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from parbot_pathplanning.srv import PARbotPathPlanning, PARbotPathPlanningResponse #import service

THRESHOLD = 50 #determines if cell is occupied before creating node
DELTA = 0.0001

def convert_occupancy_gird(grid):
    "Convert an occupancy grid to a graph"
    graphNodes = []
    queue = Queue()

    res = grid.info.resolution #meters per cell
    width = grid.info.width
    height = grid.info.height
    for y in xrange(0, height): #make grid from top left
        print "y" , y
        for x in xrange(0, width):
            current_data = ((y * width) + x)
            if grid.data[current_data] < THRESHOLD and grid.data[current_data] != -1: #check if is occupied and known
                #print "X Resolution: " , res*x
                #print 'making node'
                node = GraphNode(res*x,res*y) #create node --> from map server
                #node = GraphNode((res*x - (res*width/2)), (res*y - (res*height/2))) #create node from hector slam with negative positions
                queue.add(node) #add node to queue for Dijkstra
                #print 'Node X: ' ,  node.x , ' , Y: ' , node.y , ';'
                for other in graphNodes:
                    #print "I am here in neighbor"
                    is_neighbor(node, other, res)
                graphNodes.append(node) #this is all the nodes (never delete)

    #gridData = graphNodes, queue
    return queue

def is_neighbor(n1, n2, res):
    "determine if two nodes are connected - if so then link the nodes"
    "NOTE: Eight Connected"
    #print 'Is in_neighbhor function'
    xdiff = abs(n1.x - n2.x)
    ydiff = abs(n1.y - n2.y)
    if xdiff < (res + DELTA):
        if ydiff < (res + DELTA):
            #print 'I have made a new friend'
            Link(n1, n2)

def get_node_from_point(point, queue):
    for n in queue.list:
        xdiff = abs(n.x - point.x)
        ydiff = abs(n.y - point.y)
        if DELTA > xdiff:
            if DELTA > ydiff:
                #print "XDiff" , xdiff
                #print "YDiff" , ydiff
                print n
                if n != None:
                    return n
    print 'No Node Found' # if no node if found through the x,y checks

def path_from_grid(request):
    #returns for request
    pathExists = True #assume path exists
    waypoints = [] #empty list of points

    "return a path when a request is received"
    queue = convert_occupancy_gird(request.grid) #CHECK CASE ON occupancyGrid
    print 'Coverted occupancyGrid, looking for start'
    start = get_node_from_point(request.start, queue) #starting node
    if start == None:
        pathExists = False
    else:
        print 'Got start, looking for target'
        print start.score

    target = get_node_from_point(request.target, queue) #target node
    if target == None:
        pathExists = False
    else:
        print 'Got Target'
        print 'target score' , target.score

    if pathExists:
        score = dijkstra(queue, start, target) #run dijkstra algorthim
        path = get_path(target) #recover path though tracking node parents
        for p in path:
            waypoints.append(Point(x = p.x, y = p.y))

    return PARbotPathPlanningResponse(waypoints, pathExists)

def run():
    rospy.init_node("PARbot_Path_Planning_Server") # initialize node

    """
    service named "PARbot_Path_Planning"
    request type: PARbotPathPlanning
        request format: OccupancyGrid, start, target
            start and traget are points
    function requests are passed to: path_from_grid
    """
    s = rospy.Service('PARbot_Path_Planning', PARbotPathPlanning ,path_from_grid)
    rospy.spin() #keep code running unil service is shut down

if __name__ == "__main__":
    run()

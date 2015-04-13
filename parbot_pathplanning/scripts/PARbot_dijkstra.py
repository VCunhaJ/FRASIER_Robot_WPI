#!/usr/bin/env python

#Olivia Hugal
#Jeffrey Orszulak
#Last Revised 1/26/2014

import roslib
import rospy
from sys import maxint

DELTA = 0.0001

"NOTE: Nodes are in 8 connected form"
class GraphNode(object):
    def __init__(self, x, y, score=maxint):
       self.x = x #x cord
       self.y = y #y cord
       self.score = score
       self.neighbors = [] # a list of the node pairs (which are links)
       self.parent = None

    def get_neighbors(self):
        "Return a list of all links connecting this node"
        return self.neighbors

class Link(object):
    def __init__(self, n1, n2):
        self.n1 = n1
        self.n2 = n2

        n1.neighbors.append(self)
        n2.neighbors.append(self)

    def get_other(self, node):
        "Return the node that isn't passed in"
        return self.n1 if self.n2 == node else self.n2

class Queue:
    def __init__(self):
        self.list = []
        self.history = []

    def add(self, node):
        "Add a node if it's not already in the queue"
        if node not in self.history:
            self.list.append(node)
            self.history.append(node)

    def get_best_choice(self):
        "Returns the lowest cost node and removes it from the Queue"
        self.list.sort(cmp=self.node_cmp)
        return self.list.pop(0)

    def peek_best_choice(self):
        "Returns the lowest cost node"
        self.list.sort(cmp=self.node_cmp)
        return self.list[0]

    def exists(self):
        "Whether or not there is more to explore"
        return True if self.list else False

    def node_cmp(self, n1, n2):
        "Compares nodes by distance to target"
        return cmp(n1.score, n2.score)

"""
NOTE: Each time a node is created from the occupancy grid, it must be added
to the queue as well. This will be done in the convert_occupancy_grid function.
"""
def dijkstra(queue, start, target):
    """
    This function returns an integer if the destination is reachable or
    a string stating "Destination unreachable"
    """
    start.score = 0.0
    new_score = 0.0
    s = 'Destination unreachable'
    while queue.exists():
        u = queue.get_best_choice() #u is current node
        if u == target: #if current node is target break b/c at target
            break

        if u.score == maxint: #if no more traversible nodes end
            break

        for link in u.get_neighbors(): #for all neighbors of u, set distance score
            n = link.get_other(u)

            #diagonals for 8 connected
            xdiff = abs(n.x - u.x)
            ydiff = abs(n.y - u.y)

            if (xdiff > DELTA):
                if (ydiff > DELTA):
                    #print "Xdiff" , xdiff
                    #print "Ydiff" , ydiff
                    new_score = u.score + 1.707
            #regular 4 connected
            else:
                new_score = u.score + 1
            if new_score < n.score:
                n.score = new_score
                n.parent = u
                #Typically now would re-sort queue however get_best_choice sortsd
    if target.score == maxint:
        return s
    else:
        return target.score

def get_path(target):
    """
    This function returns the path by working backwards through the parents of
    the target. The list is then reversed.
    """
    u = target #begin list at goal location
    path = [] #empty path
    path.append(u) #add target to path

    while u.parent != None: #while node has a parent
        path.append(u.parent) #add parent to list
        u = u.parent #new current node is set to parent

    path.reverse() # reverse path to begin at start position
    return path

"For testing purposes"
if __name__ == "__main__":
    n1 = GraphNode(0, 0);   n2 = GraphNode(1, 0);  n3 = GraphNode(2, 0);  n4 = GraphNode(3, 0)
    n5 = GraphNode(0, 1);   n6 = GraphNode(1, 1);  n7 = GraphNode(2, 1);  n8 = GraphNode(3, 1)
    n9 = GraphNode(0, 2);  n10 = GraphNode(1, 2); n11 = GraphNode(2, 2); n12 = GraphNode(3, 2)
    n13 = GraphNode(0, 3); n14 = GraphNode(1, 3); n15 = GraphNode(2, 3); n16 = GraphNode(3, 3)

    Link(n1, n5); Link(n5, n6); Link(n5, n9)
    Link(n9, n13); Link(n13, n14)
    Link(n14, n15); Link(n11, n15); Link(n11, n12); Link(n15, n16); Link(n12, n16)
    Link(n12, n8); Link(n8, n4); Link(n4, n3)
    Link(n6, n10); Link(n9, n10); Link(n10, n14); Link(n10, n11)
    #Link(n7, n3); Link(n7, n6); Link(n7, n11); Link(n7, n8)
    Link(n2, n1); Link(n2, n6); Link(n2, n3)

    start = n1
    target = n8

    queue = Queue()
    queue.add(n1)
    queue.add(n2)
    queue.add(n3)
    queue.add(n4)
    queue.add(n5)
    queue.add(n6)
    queue.add(n7)
    queue.add(n8)
    queue.add(n9)
    queue.add(n10)
    queue.add(n11)
    queue.add(n12)
    queue.add(n13)
    queue.add(n14)
    queue.add(n15)
    queue.add(n16)

    score = dijkstra(queue, start, target)
    print score

    path = get_path(target)
    for n in path:
        s = 'x coord: ' + repr(n.x) + ', y coord: ' + repr(n.y)
        print s
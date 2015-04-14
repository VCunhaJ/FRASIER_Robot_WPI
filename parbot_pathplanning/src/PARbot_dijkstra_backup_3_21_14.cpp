/*
Authors: Olivia Hugal, Jeffrey Orszulak

*/

//ROS Includes
#include "ros/ros.h" //ros
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"

#include <vector>
#include <boost/foreach.hpp>
#include <stdlib.h>
#include <algorithm> //sort
#include <iostream> //cout*/
#include <string> //std::string


#define MAX_SCORE 2147483647 //maximum value used for default cost of GraphNode
#define DELTA 0.0001
#define THRESHOLD 50 //determines if cell is occupied before creating node

using namespace std;

/**
 * A class represnting locations on a map that the robot can reach.
 * Each GraphNode has an x,y coordinate and a score representing how
 * valuable of a node it is. The GraphNodes also contain a pointer to their
 * parent and a vector of their neighbors.
 */
class GraphNode
{
public:
    GraphNode(double _x, double _y){
        x = _x;
        y = _y;
        score = MAX_SCORE;
        GraphNode* parent = NULL;
    }; //constructor

    double x;
    double y;
    int score;
    vector<GraphNode*> neighbors;
    GraphNode* parent;

    /**
     * Assigns the given node as a neighbor
     * @param node
     */
    void addNeighbor(GraphNode* node){
        neighbors.push_back(node);
    }
}; //node.neighbors gets us vector

/**
 * Returns whether or not n1's score is less than n2's
 * @param  n1 The first node
 * @param  n2 The second node
 * @return    True if n1's score is less than n2's, else false
 */
bool node_cmp(GraphNode* n1, GraphNode* n2){
    return n1->score < n2->score;
}

/**
 * This is a custom priority queue sorted on the score
 * value for each GraphNode
 */
class Queue
{
public:
    vector<GraphNode*> q;//The empty queue

    /**
     * Adds a GraphNode to the Queue
     * @param q The Queue of GraphNodes
     * @param node The GraphNode to add
     */
    void add(GraphNode* node)
    {
        q.push_back(node);
    }

    /**
     * Returns the best GraphNode and removes it from the queue.
     * @param  q The queue to sort
     * @return   The best GraphNode
     */
    GraphNode* get_best_choice(){
        sort(q.begin(), q.end(), node_cmp);
        GraphNode* front = q.front();
        q.erase(q.begin()); //Removes from queue
        return front;
    }

    /**
     * Returns the best GraphNode without removing from queue
     * @param  q The queue to sort
     * @return   The best GraphNode
     */
    GraphNode* peek_best_choice(/*vector<GraphNode> q*/){
        sort(q.begin(), q.end(), node_cmp);
        return q.front();
    }

    bool empty(){
        return q.empty();
    }
};

/**
 * Sets two GraphNodes to be eachother's neighbors
 * @param n1 GraphNode 1
 * @param n2 GraphNode 2
 */
void addNeighbors(GraphNode* n1, GraphNode* n2){
    n1->addNeighbor(n2);
    n2->addNeighbor(n1);
}

/**
 * Performs Dijkstra's algorithm
 * @param q      The list of GraphNodes to navigate
 * @param start  The starting point for the algorithm
 * @param target The goal for the algorithm
 * @return       True if there is a path, else false
 */
bool Dijkstra(Queue q, GraphNode* start, GraphNode* target){
    start->score = 0; //set cost of start node to 0
    int new_score = 0;

    while (!q.empty()){
        GraphNode* u = q.get_best_choice();

        if (u->x == target->x && u->y == target->y){
            return true; //At goal
        }

        if (u->score == MAX_SCORE)
        {
            return false; //Goal unreachable
        }

        BOOST_FOREACH(GraphNode* n, u->neighbors){
            //8 connected
            double xdiff = abs(n->x - u->x);
            double ydiff = abs(n->y - u->y);

            //check diagonals for heavier weighting
            if (xdiff > DELTA)
            {
                if (ydiff > DELTA)
                {
                    new_score = u->score + 1.8; //rounded sqrt2 for diagonal distance
                }
            }
            else {
                new_score = u->score + 1; //cost of 1 for vertical or horizontal distance
            }
            if (new_score < n->score) //if found more efficient, reduce cost
            {
                n->score = new_score;
                n->parent = u;//Make a pointer u and set parent to that pointer
            }
        }
    }
}

/**
 * Returns a vector that contains the path from Dijkstra
 * @param  target The target node
 * @return        The vector of GraphNode
 */
vector<GraphNode*> get_path(GraphNode* target){
    GraphNode* u = target;
    vector<GraphNode*> path;
    path.push_back(u);

    cout<<"added target to path\n";
    while (u->parent != NULL){
        cout<<"started while loop\n";
        path.push_back(u->parent);
        cout<<"pushed\n";
        u = u->parent; //get actual value of parent
        cout<<"changed to parent\n";
        //if(u==NULL){
        //    cout<<"I am NULL\n";
        //}
        //cout<< "x coord "<< u->x <<" y coord "<< u->y <<"\n";
        cout<<"added a parent to path\n";
    }
    cout<<"created path\n";
    reverse(path.begin(), path.end());
    cout<<"Reverse path\n";
    return path;
}

/**
 * Converts an x,y coord into a ros pose
 * @param  _x     The x coord
 * @param  _y     The y coord
 * @param  _theta The angle between the current point and next point
 *                in the path
 * @return        Returns a ros PoseStamped
 */
geometry_msgs::PoseStamped get_pose(double _x, double _y, double _theta)
{
    geometry_msgs::PoseStamped poseStamped;

    //Set Header
    std::string frame ("/map");
    poseStamped.header.frame_id = frame;

    //Set Position
    poseStamped.pose.position.x = _x;
    poseStamped.pose.position.y = _y;
    poseStamped.pose.position.z = 0;
    //Empty Orientation
    poseStamped.pose.orientation.x = 0;
    poseStamped.pose.orientation.y = 0;
    poseStamped.pose.orientation.z = 0;
    poseStamped.pose.orientation.w = 0;

    return poseStamped;
}


int main(int argc, char **argv)
{
    //Set up Dijkstra
    vector<GraphNode*> path;

    //Create Nodes to test with
    GraphNode* n1 = new GraphNode(0,0);
    GraphNode* n2 = new GraphNode(1,0);
    GraphNode* n3 = new GraphNode(2,0);
    GraphNode* n4 = new GraphNode(3,0);
    GraphNode* n5 = new GraphNode(0,1);
    GraphNode* n6 = new GraphNode(1,1);
    GraphNode* n7 = new GraphNode(2,1);
    GraphNode* n8 = new GraphNode(3,1);
    GraphNode* n9 = new GraphNode(0,2);
    GraphNode* n10 = new GraphNode(1,2);
    GraphNode* n11 = new GraphNode(2,2);
    GraphNode* n12 = new GraphNode(3,2);
    GraphNode* n13 = new GraphNode(0,3);
    GraphNode* n14 = new GraphNode(1,3);
    GraphNode* n15 = new GraphNode(2,3);
    GraphNode* n16 = new GraphNode(3,3);
    cout<<"Made GraphNodes\n";

    addNeighbors(n1,n5);
    addNeighbors(n5,n6);
    addNeighbors(n5,n9);
    addNeighbors(n9,n13);
    addNeighbors(n13,n14);
    addNeighbors(n14,n15);
    addNeighbors(n11,n15);
    addNeighbors(n11,n12);
    addNeighbors(n15,n16);
    addNeighbors(n12,n16);
    addNeighbors(n12,n8);
    addNeighbors(n8,n4);
    addNeighbors(n4,n3);
    addNeighbors(n6,n10);
    addNeighbors(n9,n10);
    addNeighbors(n10,n14);
    addNeighbors(n10,n11);
    addNeighbors(n2,n1);
    addNeighbors(n2,n6);
    addNeighbors(n2,n3);
    cout<<"Added Neighbors\n";

    //Set Test Start and Target
    GraphNode* start = n1;
    GraphNode* target = n8;
    cout<<"Set Start and Target\n";

    Queue queue;
    queue.add(n1);
    queue.add(n2);
    queue.add(n3);
    queue.add(n4);
    queue.add(n5);
    queue.add(n6);
    queue.add(n7);
    queue.add(n8);
    queue.add(n9);
    queue.add(n10);
    queue.add(n11);
    queue.add(n12);
    queue.add(n13);
    queue.add(n14);
    queue.add(n15);
    queue.add(n16);
    cout<<"GraphNodes added to queue\n";

    if (Dijkstra(queue, start, target)){
        cout<<"Ran Dijkstra\n";
        path = get_path(target);
        cout<<"Got Path\n";
        BOOST_FOREACH(GraphNode* n, path){
            cout<< "x coord "<< n->x <<" y coord "<< n->y <<"\n";
        }
    }

    //Set up ROS node
    ros::init(argc, argv, "PARbot_dijkstra_node");
    ros::NodeHandle n;

    //Publisher
    ros::Publisher parbotPath_pub = n.advertise<nav_msgs::Path>("parbotPath", 1);
    ros::Rate loop_rate(1); //try to publish at 1 Hz

    // Loop, compute path, and publish
    while (ros::ok())
    {
        vector <geometry_msgs::PoseStamped> poses; //empty vector of poses
        BOOST_FOREACH(GraphNode* n, path){
            geometry_msgs::PoseStamped pose = get_pose(n->x, n->y, 0);
            poses.push_back(pose);
        }
        //Publish
        nav_msgs::Path msg; //create message of type Path
        std::string frame ("/map");
        msg.header.frame_id = frame;
        msg.poses; //array of stamped poses
        parbotPath_pub.publish(msg); //publish to topic parbotPath
        ros::spinOnce();
        loop_rate.sleep(); //wait if publish frequency is too fast
    }

    return 0;
}

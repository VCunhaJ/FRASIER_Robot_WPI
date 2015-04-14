/*
Authors: Olivia Hugal, Jeffrey Orszulak

*/

//ROS Includes
#include "ros/ros.h" //ros
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_datatypes.h"

#include <vector>
#include <boost/foreach.hpp>
#include <stdlib.h>
#include <algorithm> //sort
#include <iostream> //cout*/
#include <string> //std::string
#include <math.h> //sqrt, pow, atan2

#define MAX_SCORE 2147483647 //maximum value used for default cost of GraphNode
#define DELTA 0.0001
#define THRESHOLD 50 //determines if cell is occupied before creating node

nav_msgs::OccupancyGrid localSavedMap;

using namespace std;

//Boolean Flags
bool new_path = false; //there is a new path to publish
bool map_exists = false; //there is a map to plan on
bool new_map = false; //there is a new map to use

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
        fn = MAX_SCORE;
        gn = 0;
        GraphNode* parent = NULL;
    }; //constructor

    double x;
    double y;
    vector<GraphNode*> neighbors;
    GraphNode* parent;
    double gn, fn;
    //gn is the cost to get to where you are from the start
    //fn is the addition of gn plus the straight line distance to target

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
    return n1->fn < n2->fn;
}

//GLOBAL DECLARATION OF PATH TO PUBLISH
vector<GraphNode*> GlobalPath;

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
    GraphNode* peek_best_choice(){
        sort(q.begin(), q.end(), node_cmp);
        return q.front();
    }

    bool empty(){
        return q.empty();
    }
};

/**
 * Performs distance formula
 * @param n1     The first node
 * @param n1     The second node
 * @return       The distacne between the two points as a double
 */
double straightLineDistance(GraphNode* n1, GraphNode* n2)
{
    double dist = 0;
    dist = sqrt(pow((n1->x - n2->x),2)+pow((n1->y - n2->y),2)); //distance formula
    return dist;
}

/**
 * This is a custom priority queue sorted on the score
 * value for each GraphNode
 */
class Fringe
{
public:
    vector<GraphNode*> list;//The Fringe
    vector<GraphNode*> history;//Nodes already expanded

    /**
     * Adds a GraphNode to the Fringe
     * @param q The Fringe of GraphNodes
     * @param node The GraphNode to add
     */
    void add(GraphNode* node)
    {
        if (find(history.begin(), history.end(), node) == history.end())
        {
            list.push_back(node);
            history.push_back(node);
        }
    }

    /**
     * Returns the best GraphNode and removes it from the queue.
     * @param  q The queue to sort
     * @return   The best GraphNode
     */
    GraphNode* get_best_choice(){
        sort(list.begin(), list.end(), node_cmp);
        GraphNode* front = list.front();
        list.erase(list.begin()); //Removes from queue
        return front;
    }

    /**
     * Returns the best GraphNode without removing from queue
     * @param  q The queue to sort
     * @return   The best GraphNode
     */
    GraphNode* peek_best_choice(){
        sort(list.begin(), list.end(), node_cmp);
        return list.front();
    }

    /**
     * Expands a node for A*
     * @param node    The node to expand
     * @param target  The goal node
     */
    void expand(GraphNode* node, GraphNode* target){
        double cost, score;
        BOOST_FOREACH(GraphNode* n, node->neighbors)
        {
            cost = node->gn + straightLineDistance(node, n);
            score = straightLineDistance(n, target) + cost;
            if (score < n->fn)
            {
                n->gn = cost;
                n->fn = score;
                n->parent = node;
                add(n);
            }
        }
    }

    bool empty(){
        return list.empty();
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
 * Performs A* Search algorithm
 * @param start  The starting point for the algorithm
 * @param target The goal for the algorithm
 * @return       Return if there is a path or not
 */
bool aStar(GraphNode* start, GraphNode* target){
    cout<< "In A*\n";
    Fringe fringe;

    start->fn = straightLineDistance(start, target);
    fringe.add(start);
    while (!fringe.empty() && fringe.peek_best_choice()->fn <= target->fn)
    {
        GraphNode* node = fringe.get_best_choice();
        if ((node->x == target->x) && (node->y == target->y))
        {
            return true;
        }
        fringe.expand(node, target);
    }
    return false;
}

/**
 * Returns a vector that contains the path from A*
 * @param  start  The start node
 * @param  target The target node
 * @return        The vector of GraphNode
 */
vector<GraphNode*> get_path(GraphNode* start, GraphNode* target){
    GraphNode* u = target;
    vector<GraphNode*> path;
    path.push_back(u);

    while ((u->x != start->x) || (u->y != start->y)){
        path.push_back(u->parent);
        //cout<< "Parent: x coord "<< u->x <<" y coord "<< u->y <<"\n";
        u = u->parent; //get actual value of parent
    }

    reverse(path.begin(), path.end());
    return path;
}

double getOrientation(GraphNode* current, GraphNode* next){
    double x = next->x - current->x;
    double y = next->y - current->y;
    double desired_orientation = atan2(y, x);
    return desired_orientation;
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

    tf::Quaternion q;
    q = tf::createQuaternionFromRPY(_theta, 0, 0);

    //Set Header
    std::string frame ("/map");
    poseStamped.header.frame_id = frame;

    //Set Position
    poseStamped.pose.position.x = _x;
    poseStamped.pose.position.y = _y;
    poseStamped.pose.position.z = 0;

    poseStamped.pose.orientation.x = q.x();
    poseStamped.pose.orientation.y = q.y();
    poseStamped.pose.orientation.z = q.z();
    poseStamped.pose.orientation.w = q.w();

    /*
    //Empty Orientation
    poseStamped.pose.orientation.x = 0;
    poseStamped.pose.orientation.y = 0;
    poseStamped.pose.orientation.z = 0;
    poseStamped.pose.orientation.w = 0;
    */

    return poseStamped;
}

/**
 * Determines if two GraphNodes* are neighbors, if so,
 * they are added to eachother's neighbors list
 * @param  n1  One of the GraphNodes* to compare
 * @param  n1  One of the GraphNodes* to compare
 * @param  res Resolution of the occupancy grid being worked on
 */
void is_neighbor(GraphNode* n1, GraphNode* n2, float res)
{
    double xdiff = abs(n1->x - n2->x);
    double ydiff = abs(n1->y - n2->y);
    if (xdiff < (res + DELTA) && ydiff < (res + DELTA))
    {
        addNeighbors(n1, n2);
    }
}

/**
 * Converts an occupancy grid to a Queue to be used with Dijkstra
 * @param  grid     The published occupancy grid from /map
 * @return          Returns a Queue of GraphNodes*
 */
Queue convert_occupancy_grid(nav_msgs::OccupancyGrid grid)
{
    vector<GraphNode*> prev_row;
    vector<GraphNode*> current_row;
    Queue occupancy_queue;

    float res = grid.info.resolution; //meters per cell
    float width = grid.info.width;
    float height = grid.info.height;

    for (int y = 0; y < height; ++y)
    {
        //cout<<"y= " << y <<"\n";
        int num_node = 0; // number of nodes in this iteration of y
        int len_prev_row = prev_row.size(); //number of nodes in previous row
        for (int x = 0; x < width; ++x)
        {
            int current_data = ((y*width) + x); //may need to cast to int
            if ((grid.data[current_data] < THRESHOLD) && (grid.data[current_data] != -1))
            {
                //GraphNode* node = new GraphNode(res*x, res*y); //from map server
                GraphNode* node = new GraphNode((res*x - (res*width/2)), (res*y - (res*height/2))); //from hector slam
                occupancy_queue.add(node); //add GraphNode* to queue for Dijkstra
                for (int i = 0; i < len_prev_row; ++i) //check for neighbors in prev_row
                {
                    is_neighbor(node, prev_row[i], res);
                }
                if(num_node != 0)
                {
                    is_neighbor(node, current_row[num_node-1], res);
                }
                current_row.push_back(node); //add new node to list of nodes in this row
                num_node++; //increase number of nodes in this row
            }
        }
        if(y != (height-1)) //if this was not the last row (y) of the occupancy grid
        {
            prev_row.erase(prev_row.begin(), prev_row.end()); //empty prev_row vector
            prev_row = current_row; //copy contents of current_row into prev_row
            current_row.erase(current_row.begin(), current_row.end()); //empty current_row vector
        }
    }
    return occupancy_queue;
}

/**
 * Finds a GraphNode from a given Point
 * @param  res              The resolution of the occupancy_grid being searched
 * @param  position         The point that is being searched for.  Only x and y are used, not z
 * @param  occupancy_queue  The vector of nodes that are canidates
 * @return                  Returns GraphNode* that is closest to the given point
 */
GraphNode* get_node_from_point(float res, geometry_msgs::Point position, Queue occupancy_queue)
{
    //try to find node by checking if point is within a made grid cell in the Queue of GraphNodes
    BOOST_FOREACH(GraphNode* n, occupancy_queue.q)
    {
        double xdiff = abs(n->x - position.x);
        double ydiff = abs(n->y - position.y);
        if((DELTA + res/2) > xdiff)
        {
            if((DELTA + res/2) > ydiff)
            {
                return n;
            }
        }
    }
    //Failed to get node from point, try using distance formula
    double current_dist = 0; //stores distance to point of currently tested node
    double best_dist = 10000; //starting value for distance to best choice, will be updated with for loop
    GraphNode* best_node; //current closest node
    BOOST_FOREACH(GraphNode* n, occupancy_queue.q)
    {
        double xdiff = abs(n->x - position.x);
        double ydiff = abs(n->y - position.y);
        current_dist = sqrt(pow(xdiff,2)+pow(ydiff,2)); //distance formula
        if (current_dist < best_dist) //see if current node is better than previous best
        {
            best_dist = current_dist; //update shortest distance
            best_node = n; //update best node choice
        }
    }
    //in case of a tie, the first node found will be returned
    return best_node;
}

//CREATE START AND TARGET AS GLOBALS
geometry_msgs::Point start_point;
geometry_msgs::Point end_point;

//Make Queue as a global
Queue generatedQ;

void plan_path(){
    if(map_exists)
    {
		//Set up Search
		//Queue generatedQ;
		//geometry_msgs::Point start_point;
		//start_point.x = 0;
		//start_point.y = 0;
		//geometry_msgs::Point end_point;
		//end_point.x = 0;
		//end_point.y = 0;

		if(new_map)
		{
			generatedQ = convert_occupancy_grid(localSavedMap); //generate Queue from OccupancyGrid
			cout<<"Generated a New Queue\n";
			new_map = false;
		}

		GraphNode* start = get_node_from_point(localSavedMap.info.resolution, start_point, generatedQ); //Find Start in Queue
		cout<<"Found Start: x= " << start->x << ", y= " << start->y <<"\n";

		GraphNode* target = get_node_from_point(localSavedMap.info.resolution, end_point, generatedQ); //Find Target in Queue
		cout<<"Found Target: x= " << target->x << ", y= " << target->y <<"\n";

		if (aStar(start, target))
		{
		    cout<<"Ran Search\n";
		    GlobalPath = get_path(start, target);
			new_path = true;
		    cout<<"Got Path\n";
		    BOOST_FOREACH(GraphNode* n, GlobalPath){
		        cout<< "x coord "<< n->x <<" y coord "<< n->y <<"\n";
		    }
		}
		else{
		    cout<<"Search failed\n";
		}
    }
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    //localSavedMapPointer = msg;
    localSavedMap.header = msg->header;
    localSavedMap.info = msg->info;
    localSavedMap.data = msg->data;
    map_exists = true;
    new_map = true;
    cout << "Map resolution" << localSavedMap.info.resolution <<"\n";
}

void robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    //cout<<"Msg Location: x =" << msg->pose.position.x << ", y = " << msg->pose.position.y << "\n";
    start_point.x = msg->pose.position.x; //x location of robot as provided by message
    start_point.y = msg->pose.position.y; //y location of robot
    //cout<<"Robot Location: x =" << start_point.x << ", y = " << start_point.y << "\n";
    plan_path();
}

void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    end_point.x = msg->pose.position.x; //x location of robot as provided by message
    end_point.y = msg->pose.position.y; //y location of robot
    plan_path();

}

int main(int argc, char **argv)
{
    cout<<"setting up ROS\n";
    //Set up ROS node
    ros::init(argc, argv, "PARbot_dijkstra_node");
    ros::NodeHandle n;

    //Default Start and End points
    start_point.x = 0;
    start_point.y = 0;
    end_point.x = 0;
    end_point.y = 0;

    cout<<"setting up subsriber\n";
    //Subscribers
    ros::Subscriber Map_sub = n.subscribe("map", 1, mapCallback); //map
    ros::Subscriber SlamPose_sub = n.subscribe("robot_pose", 1, robotPoseCallback); //robot position
    ros::Subscriber TargetPose_sub = n.subscribe("Tracking/map/object_position", 1, targetPoseCallback); //target location

    cout<<"setting up publisher\n";
    //Publisher
    ros::Publisher parbotPath_pub = n.advertise<nav_msgs::Path>("parbotPath", 1);
    ros::Rate loop_rate(100); //try to publish at 100 Hz

    // Loop, compute path, and publish
    while (ros::ok())
    {
	if(new_path)
	{
		vector <geometry_msgs::PoseStamped> _poses; //empty vector of poses
		double theta = 0;
		//BOOST_FOREACH(GraphNode* n, GlobalPath){
		for (int i = 0; i < GlobalPath.size(); ++i)
		{
		    GraphNode* n = GlobalPath[i];
		    if (i < GlobalPath.size()-1)
		    {
		        theta = getOrientation(n, GlobalPath[i+1]);
		    }else{
		        theta = 0;
		    }
		    geometry_msgs::PoseStamped pose = get_pose(n->x, n->y, theta);
		    _poses.push_back(pose);
		}
		//Publish
		nav_msgs::Path msg; //create message of type Path
		std::string frame ("/map");
		msg.header.frame_id = frame;
		msg.poses = _poses; //array of stamped poses
		parbotPath_pub.publish(msg); //publish to topic parbotPath
		new_path = false;
	}
        ros::spinOnce();
        loop_rate.sleep(); //wait if publish frequency is too fast
    }

    return 0;
}

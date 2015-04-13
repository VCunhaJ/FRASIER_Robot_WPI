/*
Authors: Olivia Hugal, Jeffrey Orszulak

*/

//ROS Includes
#include "ros/ros.h" //ros
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"

#include <vector>
#include <boost/foreach.hpp>
#include <stdlib.h>
#include <algorithm> //sort
#include <iostream> //cout*/
#include <string> //std::string
#include <math.h> //sqrt, pow

#define MAX_SCORE 2147483647 //maximum value used for default cost of GraphNode
#define DELTA 0.0001
#define THRESHOLD 50 //determines if cell is occupied before creating node

//nav_msgs::OccupancyGrid::ConstPtr localSavedMapPointer;
nav_msgs::OccupancyGrid localSavedMap;

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
        hasParent=false;
    }; //constructor

    double x;
    double y;
    int score;
    vector<GraphNode*> neighbors;
    GraphNode* parent;
    bool hasParent;
    double gn, hn, fn;

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
 * Sets two GraphNodes to be eachother's neighbors
 * @param n1 GraphNode 1
 * @param n2 GraphNode 2
 */
void addNeighbors(GraphNode* n1, GraphNode* n2){
    n1->addNeighbor(n2);
    n2->addNeighbor(n1);
}


/**
 * Performs distance formula
 * @param x1     The x coordinate of the first point
 * @param y1     The y coordinate of the first point
 * @param x2     The x coordinate of the second point
 * @param y2     The y coordinate of the second point
 * @return       The distacne between the two points as a double
 */
double straightLineDistance(double x1, double y1, double x2, double y2)
{
    double dist = 0;
    dist = sqrt(pow((x1-x2),2)+pow((y1-y2),2)); //distance formula
    return dist;
}

/**
 * Performs A* Search algorithm
 * @param start  The starting point for the algorithm
 * @param target The goal for the algorithm
 * @return       Return if there is a path or not
 */
bool aStar(GraphNode* start, GraphNode* target){
    cout<<"In A*\n";

    Queue history; //list of visited nodes
    Queue fringe;  //list of nodes to evaluate
    vector<GraphNode*> parent; //map of navigated nodes
    double temp_gn;

    //Set up the starting node's values
    start->gn = 0;
    start->hn = straightLineDistance(start->x, start->y, target->x, target->y);
    start->fn = start->gn + start->hn;
    fringe.add(start); // add start node to list of points to explore

    GraphNode* current = start;
    while (!fringe.empty())
    {
        current = fringe.get_best_choice();

        if (current->x == target->x && current->y == target->y)
        {
            history.add(current);
            return true;
        }
        BOOST_FOREACH(GraphNode* n, current->neighbors)
        {
            if (find(history.q.begin(), history.q.end(), n) != history.q.end())
            {
                //continue
            }else{
                temp_gn = current->gn + straightLineDistance(current->x, current->y, n->x, n->y);

                if ((find(fringe.q.begin(), fringe.q.end(), n) == fringe.q.end()) || temp_gn < n->gn)
                {
                   n->parent = current;
                   cout << "Parent: x= " << n->parent->x << " y= " <<n->parent->y <<"\n";
                   cout << "Current: x= " << current->x << " y= " <<current->y <<"\n";
                   n->hasParent = true;
                   n->gn = temp_gn;
                   n->hn = straightLineDistance(n->x, n->y, target->x, target->y);
                   n->fn = n->gn + n->hn;
                   if (find(fringe.q.begin(), fringe.q.end(), n) == fringe.q.end())
                    {
                        fringe.add(n);
                    } 
                }
            }
        }
    }
    return false;
}

/**
 * Performs Dijkstra's algorithm
 * @param q      The list of GraphNodes to navigate
 * @param start  The starting point for the algorithm
 * @param target The goal for the algorithm
 * @return       True if there is a path, else false
 */
bool Dijkstra(Queue q, GraphNode* start, GraphNode* target){
    cout<<"In Dijkstra\n";

    start->score = 0; //set cost of start node to 0
    int new_score = 0;
    while (!q.empty())
    {
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
                n->hasParent = true;
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

    while (u->hasParent){
        path.push_back(u->parent);
        //cout<< "Parent: x coord "<< u->x <<" y coord "<< u->y <<"\n";
        u = u->parent; //get actual value of parent
    }

    reverse(path.begin(), path.end());
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
        cout<<"y= " << y <<"\n";
        int num_node = 0; // number of nodes in this iteration of y
        int len_prev_row = prev_row.size(); //number of nodes in previous row
        for (int x = 0; x < width; ++x)
        {
            int current_data = ((y*width) + x); //may need to cast to int
            if ((grid.data[current_data] < THRESHOLD) && (grid.data[current_data] != -1))
            {
               GraphNode* node = new GraphNode(res*x, res*y); //from map server
                //GraphNode* node = new GraphNode((res*x - (res*width/2)), (res*y - (res*height/2))); //from hector slam
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

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    //localSavedMapPointer = msg;
    localSavedMap.header = msg->header;
    localSavedMap.info = msg->info;
    localSavedMap.data = msg->data;
    cout << "Map resolution" << localSavedMap.info.resolution <<"\n";

    //Set up Dijkstra
    Queue generatedQ;
    geometry_msgs::Point start_point;
    start_point.x = 5;
    start_point.y = 5;   
    geometry_msgs::Point end_point;
    end_point.x = 6;
    end_point.y = 6;

    generatedQ = convert_occupancy_grid(localSavedMap); //generate Queue from OccupancyGrid
    cout<<"Generated Queue\n";

    GraphNode* start = get_node_from_point(localSavedMap.info.resolution, start_point, generatedQ); //Find Start in Queue
    cout<<"Found Start: x= " << start->x << ", y= " << start->y <<"\n";

    GraphNode* target = get_node_from_point(localSavedMap.info.resolution, end_point, generatedQ); //Find Target in Queue
    cout<<"Found Target: x= " << target->x << ", y= " << target->y <<"\n";

    //run Dijkstra to get path
    //if (Dijkstra(generatedQ, start, target))
    if (aStar(start, target))
    {
        cout<<"Ran Search\n";
        GlobalPath = get_path(target);
        cout<<"Got Path\n";
        BOOST_FOREACH(GraphNode* n, GlobalPath){
            cout<< "x coord "<< n->x <<" y coord "<< n->y <<"\n";
        }
    }
}

int main(int argc, char **argv)
{
    cout<<"setting up ROS\n";
    //Set up ROS node
    ros::init(argc, argv, "PARbot_dijkstra_node");
    ros::NodeHandle n;

    cout<<"setting up subsriber\n";
    //Subscriber
    ros::Subscriber Map_sub = n.subscribe("map", 1, mapCallback);

    cout<<"setting up publisher\n";
    //Publisher
    ros::Publisher parbotPath_pub = n.advertise<nav_msgs::Path>("parbotPath", 1);
    ros::Rate loop_rate(1); //try to publish at 1 Hz

    // Loop, compute path, and publish
    while (ros::ok())
    {
        vector <geometry_msgs::PoseStamped> _poses; //empty vector of poses
        BOOST_FOREACH(GraphNode* n, GlobalPath){
            geometry_msgs::PoseStamped pose = get_pose(n->x, n->y, 0);
            _poses.push_back(pose);
        }
        //Publish
        nav_msgs::Path msg; //create message of type Path
        std::string frame ("/map");
        msg.header.frame_id = frame;
        msg.poses = _poses; //array of stamped poses
        parbotPath_pub.publish(msg); //publish to topic parbotPath
        ros::spinOnce();
        loop_rate.sleep(); //wait if publish frequency is too fast
    }

    return 0;
}

Olivia Hugal
Jeffrey Orszulak

Running in Terminal:


TODO:

Testing to do:
    test PARbot_display path
        switch to small map, dont use hector slam and gazebo first

    to make map work for all cases subtract origin postion, not two lines as is no (origin in map meta data)
    Take into account negative coordinates in map -- sort of works
    Need to do c space
    Need to have grid update when occupancy grid size changes
        Once map is really large, node creation may be slow
    Add cases to map for unreachable targets
    Deal with empty maps
    fix diagonals

    !!!!!Change geometrt_msgs/Point[] waypoints to nav_msgs/Path.msg leave orientation blank

roscore

START MAP SERVER - must be in parbot_pathplanning folder
rosrun map_server map_server rbe3002test-small.yaml

START RVIZ
rosrun rviz rviz

START PATH PLANNING SERVICE
rosrun parbot_pathplanning PARbot_dijkstra_on_occupancy_grid.py

MAKE CALL FOR PATH
rosrun parbot_pathplanning PARbot_dijkstra_return_path.py

GAZEBO: turtle
roslaunch turtlebot_gazebo turtlebot_willow_garage.launch
roslaunch turtlebot_gazebo gmapping_demo.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch

GAZEBO: husky
http://www.clearpathrobotics.com/blog/husky-simulation-in-gazebo/
roslaunch husky_gazebo willowgarage_world.launch
roslaunch husky_teleop teleop.launch
roslaunch husky_viz view_robot.launch
    make sure to add laserscan

HECTOR SLAM:
roslaunch parbot_pathplanning PARbot_slam.launch
http://answers.ros.org/question/95486/hector-slam-with-turtlebot-in-gazebo/

FROM KEVIN:
the commands you need are
ssh -X parbot@192.168.1.100

sshed into the robot
roslaunch parbot_master parbot_master_bringup.launch
rosrun parbot_teleop parbot_teleop_node
and on your local machine
rosrun joy joy_node


Laser Scan topic from PARbot: base_scan

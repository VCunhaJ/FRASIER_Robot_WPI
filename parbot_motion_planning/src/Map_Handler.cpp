#include "Map_Handler.h"

Map_Handler::Map_Handler(ros::NodeHandle n)
{
    map_sub = n.subscribe(std::string(SUB_TOPIC), 1, &Map_Handler::mapCallback, this);
    points = std::vector<grid_cart_point>(1);
	got_map = false;
};

std::vector<grid_cart_point> Map_Handler::get_points()
{
    return points;
}

std::vector<grid_cart_point> Map_Handler::convert_occupancy_grid(nav_msgs::OccupancyGrid grid)
{
    int obstacles = 0; // number of nodes in this iteration of y
    float res = grid.info.resolution; //meters per cell
    float width = grid.info.width;
    float height = grid.info.height;
    std::vector<grid_cart_point> all_obstacles = std::vector<grid_cart_point>(width*height);

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int current_data = ((y*width) + x); //may need to cast to int
            if ((grid.data[current_data] > THRESHOLD) && (grid.data[current_data] != -1))
            {
                cart_point point;
                point.x = (res*x - (res*width/2));
                point.y = (res*y - (res*height/2));

                grid_cart_point g_point = cart_to_grid(point);

                all_obstacles[obstacles] = g_point;
                obstacles++; //increase number o  in this row
            }
        }
    }
	//ROS_INFO("%d points converted", obstacles);
    all_obstacles.resize(obstacles);
    return all_obstacles;
}

void Map_Handler::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    //ROS_INFO("map received");
    nav_msgs::OccupancyGrid map;
    map.info = msg->info;
    map.data = msg->data;
    map.header = msg->header;
    points = convert_occupancy_grid(map);
	if(!got_map)
		got_map = true;
}

bool Map_Handler::ready()
{
	return got_map;
}

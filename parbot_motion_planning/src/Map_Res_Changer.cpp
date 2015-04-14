#include "Map_Res_Changer.h"

//DEVELOPMENT STOPPED UNEEDED

Map_Res_Changer::Map_Res_Changer(double _target_res)
{
    target_res = _target_res;
    fp = new Square_Grid_Footprint(target_res, target_res);
};

nav_msgs::OccupancyGrid Map_Res_Changer::modify_map(nav_msgs::OccupancyGrid map_msg, double _target_res)
{
    if(_target_res != target_res)
    {
        target_res = _target_res;
    }

    if(fp->get_target_res() != target_res || fp->get_source_res() != map_msg.info.resolution)
    {
        fp = new Square_Grid_Footprint(target_res, map_msg.info.resolution);
    }

    std::vector<cart_point> points = convert_occupancy_grid(map_msg);
    return NULL;
}

std::vector<cost_point> Map_Res_Changer::convert_occupancy_grid(nav_msgs::OccupancyGrid grid)
{
    int obstacles = 0; // number of nodes in this iteration of y
    float res = grid.info.resolution; //meters per cell
    float width = grid.info.width;
    float height = grid.info.height;
    std::vector<cost_point> all_obstacles = std::vector<cost_point>(width*height);

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

                cost_point c_point;
                c_point.point=point;
                c_point.cost=grid.data[current_data];

                all_obstacles[obstacles] = c_point;
                obstacles++; //increase number o  in this row
            }
        }
    }
    all_obstacles.resize(obstacles);
    return all_obstacles;
}

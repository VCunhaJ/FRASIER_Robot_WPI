#include "motion_planning_structures.h"
#include "nav_msgs/ccupancyGrid.h"

//DEVELOPMENT STOPPED UNEEDED

class Map_Res_Changer{
public:
    Map_Res_Changer(double _target_res);
    nav_msgs::OccupancyGrid modify_map(nav_msgs::OccupancyGrid map_msg, double _target_res=target_res);
private:
    double target_res;
    Square_Grid_Footprint* fp;
    nav_msgs::OccupancyGrid new_map;

    std::vector<cost_point> convert_occupancy_grid(nav_msgs::OccupancyGrid grid);

};

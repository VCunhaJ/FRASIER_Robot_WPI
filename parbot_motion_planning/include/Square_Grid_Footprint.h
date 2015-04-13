#include "motion_planning_structures.h"

#define THRESHOLD 50

class Square_Grid_Footprint{
public:
    Square_Grid_Footprint(double _target_grid_size, double _source_grid_size);
    std::vector<cart_point> points_from_point(cart_point point, double _source_grid_size);
    std::vector<cart_point> points_from_point(cart_point point);
    std::vector<std::vector<cart_point> > points_from_points(std::vector<cart_point> points);
    double get_target_res();
    double get_source_res();
private:
    double source_grid_size, target_grid_size;
    std::vector<cart_point> foot_print;

    void make_footprint();
    cart_point move_point(cart_point local_point, cart_point target_point);
};

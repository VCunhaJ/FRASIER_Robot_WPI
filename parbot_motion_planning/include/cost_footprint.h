#ifndef COST_FOOTPRINT_H
#define COST_FOOTPRINT_H

#include "motion_planning_structures.h"

#define DEFAULT_COST 10
#define DEFAULT_R 0.5

class cost_footprint{
public:
    cost_footprint(double _c_max=DEFAULT_COST, double _r_max=DEFAULT_R, double _grid_size=GRID_SIZE);

    std::vector<cost_grid_point> cost_points;
    double c_max, r_max, grid_size;

    void make_grid();
    double cost_fun(double r);
    std::vector<cost_grid_point> points_from_point(grid_cart_point start_point);
    bool is_unique(cost_polar_point point, std::vector<cost_polar_point> cost_points);
    void print_footprint();
};

#endif

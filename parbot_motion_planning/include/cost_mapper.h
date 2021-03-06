#ifndef COST_MAPPER_H
#define COST_MAPPER_H

#include "motion_planning_structures.h"
#include "cost_footprint.h"
#include "parbot_motion_planning/cost_map.h"
#include "parbot_motion_planning/cost_map_2.h"
#include "Map_Data.h"

class cost_mapper{
public:
    cost_mapper(double _c_max=DEFAULT_COST, double _r_max=DEFAULT_R, double _grid_size=GRID_SIZE);

    void update_cost_map(std::vector<grid_cart_point> object_points);
	void update_cost_map(std::vector<grid_cart_point> object_points, double size);
    void update_cost_map(Map_Data* md);

    std::vector<cost_grid_point> get_cost_map();
    double score_path(std::vector<cart_point> robot_path);
    void print_cost_grid();
    void print_cost_grid(std::vector<grid_cart_point> obstacle_points);
    void print_cost_grid(std::vector<grid_cart_point> obstacle_points, std::vector<cart_point> path_points);
    parbot_motion_planning::cost_map get_map();
    parbot_motion_planning::cost_map_2 get_map_2();
private:
    cost_footprint* cfp;
    std::vector<cost_grid_point> cost_map;
    int cost_grid_offset_x, cost_grid_offset_y;
    std::vector<std::vector<double> > cost_grid;
    double grid_size, cost, radius;
    parbot_motion_planning::cost_map map;
    parbot_motion_planning::cost_map_2 map_2;

    void make_cost_grid(std::vector<grid_cart_point> points, cost_footprint* fp);
    void make_cost_grid(std::vector<grid_cart_point> points);
    void make_cost_grid(Map_Data* md);
    void add_to_grid(cost_grid_point point);
    double lookup_point(cart_point c_point);
    void init_grid(int x, int y);
    std::vector<std::vector<cost_grid_point> > make_cost_points(std::vector<grid_cart_point> grid_points);
    void add_points_to_grid(std::vector<grid_cart_point> points);
    void add_points_to_grid(std::vector<std::vector<cost_grid_point> > cost_points);
    void init_map();
    void init_map_2();
    void init_grid(std::vector<std::vector<double> > grid);
	void csv_print();
};

#endif

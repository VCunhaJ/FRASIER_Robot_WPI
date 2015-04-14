#ifndef TENTACLE_H
#define TENTACLE_H

#include <vector>
#include "motion_planning_structures.h"
#include "motion_planning_defs.h"
#include "robot_footprint.h"
#include "cost_mapper.h"

#define COST_WEIGHT 0.05//1.25
#define PATH_WEIGHT 1
#define LENG_CONST 1	

class tentacle
{
public:
	tentacle(double _s, double _theta, robot_footprint* _robo_foot, double _grid_size=GRID_SIZE, double _width=WIDTH);


	int sign, lookup_offset_x, lookup_offset_y;
	//std::vector<cart_point> collision_points;
	std::vector<cart_point> path_points;
	std::vector<cart_pose> path_poses;
	std::vector<grid_cart_point> grid_collision_points;
	robot_footprint* robo_foot;
	std::vector<std::vector<int> > look_up_grid;
	double score;

	std::vector<cart_point> make_collision_box();
	std::vector<cart_point> make_straight_collision_box();
	std::vector<cart_pose> make_path_straight();
	std::vector<cart_pose> make_path();
	std::vector<grid_cart_point> make_grid_collision_box();
	bool is_safe(std::vector<grid_cart_point> collision_points);
	void print_grid();
	void print_grid_inline(int index);
	void print_grid(std::vector<grid_cart_point> obstacle_points);
	void print_grid(std::vector<grid_cart_point> obstacle_points, std::vector<cart_point> target_path);
	void print_collision();
	void print_path();
	void print_path(std::vector<grid_cart_point> obstacle_points, std::vector<cart_point> target_path);
	void make_lookup_grid();
	void add_to_lookup_grid(grid_cart_point point);
	void print_lookup_grid();
	bool check_lookup(grid_cart_point point);
    void set_cost_score(std::vector<cost_grid_point> cost_grid);
	std::vector<grid_cart_point> get_grid_from_lookup();
	double set_score(std::vector<cart_point> robot_path);//returns score and saves it with tentacle
	double get_v();
	double get_w();
    double straight_line_dist(cart_point c1, cart_point c2);
    void update_combined_score();
    double get_cost_score();
    double get_combined_score();
    void update_scores(std::vector<cart_point> robot_path, std::vector<cost_grid_point> cost_grid);
    void update_scores(std::vector<cart_point> robot_path,
        cost_mapper mapper);
    double get_score();
private:
	double v,w,theta,s,r, width, grid_size, combined_score, cost_score;
};
#endif

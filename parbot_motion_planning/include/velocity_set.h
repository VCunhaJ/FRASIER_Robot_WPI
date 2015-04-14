#ifndef VELOCITY_SET_H
#define VELOCITY_SET_H

#include "tentacle.h"

class velocity_set{
public:
	velocity_set(double _v, std::vector<double> thetas, robot_footprint* robot_foot, double _width=WIDTH,
		double _grid_size=GRID_SIZE);

	std::vector<tentacle*> tentacles;
	double v, width, grid_size;
	std::vector<tentacle*> safe_tentacles;

	void make_tentacles(std::vector<double> thetas, robot_footprint* robot_foot);
	void print_collision();
	void print_grid();
	void print_grid(std::vector<tentacle*> tents);
	void print_grid_inline(int index);
	void print_path();
	void print_path_inline(int index);
	std::vector<tentacle*> make_safe_tentacles(std::vector<grid_cart_point> scan_points);
	std::vector<tentacle*> get_safe_tentacles();
	int len_grid_points();
	int len_grid_points(std::vector<tentacle*> tents);
	void update_safe_tentacles(std::vector<grid_cart_point> scan_point);
	void print_safe_grid();
	void print_safe_paths();
	void print_safe_paths_inline(int index);
	void print_safe_grid(std::vector<grid_cart_point> collision_points);
	void print_safe_grid_inline(int index);
	std::vector<std::vector<grid_cart_point> > get_collision_grids();
	bool has_safe_tentacle(std::vector<grid_cart_point> obstacle_points);
	int len_path_points();
	void print_paths_grid_inline(int index);
	void print_paths_grid_endline(int index);
	tentacle* lowest_safe_score(std::vector<cart_point> robot_path);
    std::vector<tentacle*> lowest_safe_score(std::vector<cart_point> robot_path, int num_to_find);
    tentacle* lowest_safe_score(std::vector<cart_point> robot_path, std::vector<cost_grid_point> cost_map);
    tentacle* lowest_safe_score(std::vector<cart_point> robot_path, cost_mapper mapper);
};

#endif

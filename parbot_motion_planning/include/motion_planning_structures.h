#ifndef motion_planning_strct
#define motion_planning_strct

#include <iostream>
#include <vector>
#include "motion_planning_defs.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ros/ros.h"
#include <omp.h>

struct cart_point{
	double x,y;};

struct polar_point{
    double r,theta;
};

struct motion_path{
	std::vector<cart_point> collision_box;
	std::vector<cart_point> center_path;
	double v,w;
};

struct grid_cart_point{
	int x,y;};

struct cart_pose{
	cart_point point;
	double theta;
};

struct cost_grid_point{
    grid_cart_point point;
    double cost;
};

struct cost_point{
    cart_point point;
    double cost;
};

struct cost_polar_point{
    polar_point point;
    double cost;
};

double round(double val);
void print_cart_point(cart_point point);
void print_cart_point_inline(cart_point point);
void print_cart_point_endline(cart_point point);
void print_grid_cart_point(grid_cart_point point);
void print_grid_cart_point_inline(grid_cart_point point);
void print_grid_cart_point_endline(grid_cart_point point);
grid_cart_point cart_to_grid(cart_point c_point, double grid_size=GRID_SIZE);
cart_point grid_to_cart(grid_cart_point g_point, double grid_size=GRID_SIZE);
bool grid_cart_points_equal(grid_cart_point p1, grid_cart_point p2);
grid_cart_point polar_to_grid(polar_point p_point);
cart_point polar_to_cart(polar_point p_point);
polar_point cart_to_polar(cart_point c_point);
void print_cart_points(std::vector<cart_point> points);
void print_grid_points(std::vector<grid_cart_point> points);
std::vector<cart_point> grid_to_cart(std::vector<grid_cart_point> g_points, double grid_size=GRID_SIZE);
std::vector<grid_cart_point> polar_to_grid(std::vector<polar_point> polar_points);
std::vector<cart_point> polar_to_cart(
    std::vector<polar_point> p_points);
std::vector<grid_cart_point> cart_to_grid(std::vector<cart_point> c_points);
double stl(cart_point c1, cart_point c2);
std::vector<cost_grid_point> polar_cost_grid(std::vector<cost_polar_point> polar_points);
cost_grid_point polar_cost_grid(cost_polar_point p_point);
std::vector<cart_point> cost_to_cart(std::vector<cost_point> cost_points);
std::vector<grid_cart_point> merge_unique(std::vector<grid_cart_point> points1, std::vector<grid_cart_point> points2);
bool unique_point(grid_cart_point point, std::vector<grid_cart_point> points);
#endif

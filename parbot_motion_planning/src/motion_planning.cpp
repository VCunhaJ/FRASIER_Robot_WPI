// motion_planning.cpp : Defines the entry point for the console application.
//
#include <math.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <ctime>
#include "motion_planning_structures.h"
#include "motion_planning_defs.h"
#include "velocity_set.h"
#include "motion_planner.h"
#include "robot_footprint.h"
#include <time.h>

//takes r, theta, -> creates line in grid, with size grid_size
//assumes standard ros coordinate system of x z up etc
std::vector<cart_point> arc_grid_area(double r, double theta_max, int sign=1, double width=WIDTH, double grid_size=GRID_SIZE)
{
	double s = r*theta_max;
	int n = abs(s/grid_size);
	double delta_theta = abs(theta_max/n);
	int p = abs(width/grid_size);
	std::vector<cart_point> points = std::vector<cart_point>(n*(p));
	for(int i=0; i<n; i++)
	{
		double theta = delta_theta*i;
		double tmp_r = r - width/2 - grid_size;
		for(int j=0; j<p; j++)
		{
			tmp_r += grid_size;
			cart_point point;
			point.x = tmp_r*sin(theta);
			point.y = sign*(r-tmp_r*cos(theta));
			points[i*p+j] = point;
		}
	}

	return points;
}

std::vector<cart_point> straight_path_area(double length, int sign=1, double width=WIDTH, double grid_size=GRID_SIZE)
{
	int num_x = length / grid_size;
	int num_y = width / grid_size;
	double delta_x = grid_size;
	double delta_y = grid_size;
	double current_x = 0;
	std::vector<cart_point> points = std::vector<cart_point>(num_x*num_y);
	for(int i=0; i<num_x; i++)
	{
		double current_y = -1*width/2;
		for(int j=0; j<num_y; j++)
		{
			cart_point point;
			point.x = sign*delta_x*i;
			point.y = sign*delta_y*j-width/2;
			points[i*num_y+j] = point;
		}
	}

	return points;
}

void print_cart_vector(std::vector<cart_point> points)
{
	for(int i=0; i<points.size(); i++)
	{
		std::cout << points[i].y << "," << points[i].x << "\n";
	}
}

int max(int a, int b)
{
	if(b < a)
		return a;
	else
		return b;
}

void print_all_points_center(std::vector<motion_path> set_paths)
{
	int num_areas = set_paths.size();
	for(int i=0; i<set_paths[0].center_path.size(); i++)
	{
		for(int j=0; j<set_paths.size(); j++)
		{
			if(j < set_paths.size() - 1)
				print_cart_point_inline(set_paths[j].center_path[i]);
			else
				print_cart_point_endline(set_paths[j].center_path[i]);
		}
	}
}

void print_all_points_collision(std::vector<motion_path> set_paths)
{
	int num_areas = set_paths.size();
	for(int i=0; i<set_paths[0].collision_box.size(); i++)
	{
		for(int j=0; j<set_paths.size(); j++)
		{
			if(j < set_paths.size() - 1)
				print_cart_point_inline(set_paths[j].collision_box[i]);
			else
				print_cart_point_endline(set_paths[j].collision_box[i]);
		}
	}
}

std::vector<motion_path> make_tentacle_collision_boxes(double s, std::vector<double> thetas)
{
	int num_tentacles = thetas.size();
	std::vector<motion_path> tentacles = std::vector<motion_path>(2*num_tentacles+1);
	for(int i=0; i<num_tentacles; i++)
	{
		if(thetas[i] == 0)
			continue;
		double r = s/thetas[i];
		motion_path path,opp_path;
		path.collision_box = arc_grid_area(r, thetas[i]);
		opp_path.collision_box = arc_grid_area(r, thetas[i], -1);
		path.v = s;
		path.w = thetas[i];
		opp_path.v = s;
		opp_path.w = -1*thetas[i];
		tentacles[i] = path;
		tentacles[i + 1 + num_tentacles] = opp_path;
	}
	tentacles[num_tentacles].collision_box = straight_path_area(s);
	return tentacles;
}

std::vector<cart_point> straight_centerline_path(double length, int sign=1, double grid_size=GRID_SIZE)
{
	int num_x = length / grid_size;
	double delta_x = grid_size;
	double delta_y = grid_size;
	double current_y = 0;
	std::vector<cart_point> points = std::vector<cart_point>(num_x);
	for(int i=0; i<num_x; i++)
	{
		cart_point point;
		point.x = sign*delta_x*i;
		point.y = current_y;
		points[i] = point;
	}

	return points;
}

std::vector<cart_point> arc_centerpath(double r, double theta_max, int sign=1, double grid_size=GRID_SIZE)
{
	double s = r*theta_max;
	int n = abs(s/grid_size);
	double delta_theta = abs(theta_max/n);
	std::vector<cart_point> points = std::vector<cart_point>(n);
	for(int i=0; i<n; i++)
	{
		double theta = delta_theta*i;
		cart_point point;
		point.x = r*sin(theta);
		point.y = sign*(r-r*cos(theta));
		points[i] = point;
	}

	return points;
}

std::vector<motion_path> make_tentacle_centerlines(double s, std::vector<double> thetas)
{
	int num_tentacles = thetas.size();
	std::vector<motion_path> tentacles = std::vector<motion_path>(2*num_tentacles+1);
	for(int i=0; i<num_tentacles; i++)
	{
		if(thetas[i] == 0)
			continue;
		double r = s/thetas[i];
		motion_path path,opp_path;
		path.center_path = arc_centerpath(r, thetas[i]);
		path.v = s;
		path.w = thetas[i];
		opp_path.center_path = arc_centerpath(r, thetas[i], -1);
		opp_path.v = s;
		opp_path.w = -1*thetas[i];
		tentacles[i] = path;
		tentacles[i + 1 + num_tentacles] = opp_path;
	}
	tentacles[num_tentacles].center_path = straight_centerline_path(s);
	tentacles[num_tentacles].v = s;
	tentacles[num_tentacles].w = 0;
	return tentacles;
}

std::vector<motion_path> make_tentacles(double s, std::vector<double> thetas)
{
	std::vector<motion_path> tentacle_centers = make_tentacle_centerlines(s, thetas);
	std::vector<motion_path> tentacle_boxes = make_tentacle_collision_boxes(s,thetas);
	std::vector<motion_path> tentacles = std::vector<motion_path>(tentacle_boxes.size());
	for(int i=0; i<tentacle_boxes.size(); i++)
	{
		tentacles[i].center_path = tentacle_centers[i].center_path;
		tentacles[i].collision_box = tentacle_boxes[i].collision_box;
		tentacles[i].v = tentacle_centers[i].v;
		tentacles[i].w = tentacle_centers[i].w;
	}

	return tentacles;
}

std::vector<grid_cart_point> offset_grid_line(double s, double offset,
	int sign=1, double grid_size=GRID_SIZE)
{
	int num_x = s / grid_size;
	double delta_x = grid_size;
	double delta_y = grid_size;
	double current_x = 0;
	double current_y = offset;
	std::vector<grid_cart_point> points = std::vector<grid_cart_point>(num_x);
	for(int i=0; i<num_x; i++)
	{
		cart_point point;
		point.x = delta_x*i;
		point.y = current_y;
		points[i] = cart_to_grid(point);
	}


	return points;
}

std::vector<grid_cart_point> horizontal_grid_line(double len, double offset_y, double offset_x,
	double width=WIDTH, int sign=1, double grid_size=GRID_SIZE)
{
	int num_y = len / grid_size;
	int num_x = width / grid_size;
	double delta_x = 0;
	double delta_y = grid_size;
	double current_x = offset_x;
	double current_y = offset_y;
	std::vector<grid_cart_point> points = std::vector<grid_cart_point>(num_y);
	for(int i=0; i<num_y; i++)
	{
		cart_point point;
		point.x = current_x + delta_x*i;
		point.y = current_y + delta_y*i;
		points[i] = cart_to_grid(point);
	}
	return points;
}

void print_grid_line(std::vector<grid_cart_point> points)
{
	for(int i=0; i<points.size(); i++)
	{
		print_grid_cart_point_endline(points[i]);
	}
}

std::vector<grid_cart_point> merge_grid_lines(std::vector<grid_cart_point> line1,
	std::vector<grid_cart_point> line2)
{
	int len = line1.size() + line2.size();
	std::vector<grid_cart_point> merged_line = std::vector<grid_cart_point>(len);
	for(int i=0; i<line1.size(); i++)
	{
		merged_line[i] = line1[i];
	}
	for(int i=line1.size(); i<len; i++)
	{
		merged_line[i] = line2[i-line1.size()];
	}
	return merged_line;
}

std::vector<cart_point> merge_lines(std::vector<cart_point> line1, std::vector<cart_point> line2)
{
	int len = line1.size() + line2.size();
	std::vector<cart_point> merged_line = std::vector<cart_point>(len);
	for(int i=0; i<line1.size(); i++)
	{
		merged_line[i] = line1[i];
	}
	for(int i=line1.size(); i<len; i++)
	{
		merged_line[i] = line2[i-line1.size()];
	}
	return merged_line;
}

std::vector<cart_point> concat(std::vector<cart_point> ps1, std::vector<cart_point> ps2)
{
	std::vector<cart_point> cat_v = std::vector<cart_point>(ps1.size()+ps2.size());
	for(int i=0; i<ps1.size(); i++)
	{

cat_v[i] = ps1[i];
	}
	for(int i=ps1.size(); i<cat_v.size(); i++)
	{
		cat_v[i] = ps2[i-ps1.size()];
	}
	return cat_v;
}



int main(int argc, char **argv)
{
	std::vector<grid_cart_point> h_line = horizontal_grid_line(2, -1, 2);
	std::vector<grid_cart_point> v_line = offset_grid_line(2, 1);
	std::vector<grid_cart_point> impact_line = merge_grid_lines(h_line, v_line);

	robot_footprint* robot_foot = new robot_footprint();

	tentacle* test_tent = new tentacle(0.5, 0, robot_foot, 0.2);

	std::vector<cart_point> target_path = test_tent->path_points;//straight_centerline_path(1);

    //ROS_INFO("self score %f", test_tent->set_score(test_tent->path_points));
    //test_tent->print_path(test_tent->get_grid_from_lookup(), test_tent->path_points);
    /*
	std::vector<double> thetas = std::vector<double>(NUM_THETAS);
	int i;
	for(i=0; i<NUM_THETAS; i++)
	{
		thetas[i] = THETA_INIT-DELTA_THETA*i;
	}

	double v_step = (V_MAX-V_MIN)/(NUM_VELOCITIES-1);
 	std::vector<double> velocities = std::vector<double>(NUM_VELOCITIES);
	for(int i=0; i<NUM_VELOCITIES; i++)
	{
		velocities[i] = V_MAX-v_step*i;
        ROS_INFO("V set for %f", velocities[i]);
	}
    */
	ros::init(argc, argv, "motion_planner_test");
	ros::NodeHandle n;
	ros::Time begin = ros::Time::now();
	motion_planner* planner = new motion_planner(n);
    //ROS_INFO("init complete");
	//planner->lsh->scan_points = impact_line;
	//planner->pah->robot_path = target_path;
	//planner->pah->message_received = true;
    //ROS_INFO("string to wait");
    //planner->waitForReady();
	//ROS_INFO("done waiting");
	//planner->plan();
	//ROS_INFO("plan complete");

    //planner->update_obstacle_points();
//*
    planner->waitForReady();
    ros::Rate loop_rate(10);
    for(int i=0; i<1; i++)
    {
        planner->plan_once();
        ros::spinOnce();
        loop_rate.sleep();
    }
//*/
    //ROS_INFO("about to make cost_map");
    //ros::Time begin = ros::Time::now();
    //planner->cost_map.update_cost_map(planner->obstacle_points);
    ros::Time end = ros::Time::now();

    //planner->cost_map.print_cost_grid(planner->obstacle_points);


	double elapsed = ((double)end.sec-(double)begin.sec)+((double)end.nsec-(double)begin.nsec)/1000000000;

	//ROS_INFO("Planner test time = %f s", elapsed);

 	return 0;
}


#include "tentacle.h"


tentacle::tentacle(double _s, double _theta, robot_footprint* _robo_foot, double _grid_size, double _width)//assumes v = s ie -> calculating for one second ahead
{
	s = _s;
	v = s;
	w = (v/s)*_theta;
    //ROS_INFO("theta = %f", w);
	theta = fabs(_theta);
	if(_theta < 0)
		sign = -1;
	else
		sign = 1;
	r = s/theta;
	grid_size = _grid_size;
	width = _width;
	robo_foot = _robo_foot;
	make_lookup_grid();
	score = -1;
    cost_score = -1;
};


//assumes standard ros coordinate system of x z up etc
std::vector<cart_point> tentacle::make_collision_box()
{
	std::vector<cart_pose> path_poses;
	if(theta == 0)
		path_poses = make_path_straight();
	else
		path_poses = make_path();
	std::vector<cart_point> collision_points = std::vector<cart_point>(path_poses.size()*robo_foot->foot_print.size());
	for(int i=0; i<path_poses.size(); i++)
	{
		std::vector<cart_point> rotated_foot = robo_foot->foot_print_at(path_poses[i].theta);
		for(int j=0; j<robo_foot->foot_print.size(); j++)
		{
			cart_point new_point;
			new_point.x = path_poses[i].point.x + rotated_foot[j].x;
			new_point.y = path_poses[i].point.y + rotated_foot[j].y;
			collision_points[i*robo_foot->foot_print.size()+j] = new_point;
		}
	}
	return collision_points;
}

std::vector<cart_point> tentacle::make_straight_collision_box()
{
	int num_x = s / grid_size;
	int num_y = width / grid_size;
	double delta_x = grid_size;
	double delta_y = grid_size;
	double current_x = 0;
	std::vector<cart_point> points = std::vector<cart_point>(num_x*num_y);
	path_points = std::vector<cart_point>(num_x);
	for(int i=0; i<num_x; i++)
	{
		double current_y = -1*width/2;
		cart_point path_point;
		path_point.x = delta_x*i;
		path_point.y = 0;
		path_points[i] = path_point;
		for(int j=0; j<num_y; j++)
		{
			cart_point point;
			point.x = delta_x*i;
			point.y = delta_y*j-width/2;
			points[i*num_y+j] = point;
		}
	}

	return points;
}

std::vector<cart_pose> tentacle::make_path_straight()
{
	int num_x = s / grid_size;
	double delta_x = grid_size;
	double delta_y = grid_size;
	double current_y = 0;
	path_points = std::vector<cart_point>(num_x);
	path_poses = std::vector<cart_pose>(num_x);
	for(int i=0; i<num_x; i++)
	{
		cart_point point;
		point.x = delta_x*i;
		point.y = current_y;
		path_points[i] = point;
		cart_pose pose;
		pose.point = point;
		pose.theta = 0;
		path_poses[i] = pose;
	}
	return path_poses;
}

/*
void tentacle::make_path()
{
	double s = r*theta;
	int n = abs(s/grid_size);
	double delta_theta = abs(theta/n);
	std::vector<cart_point> points = std::vector<cart_point>(n);
	for(int i=0; i<n; i++)
	{
		double theta = delta_theta*i;
		cart_point point;
		point.x = r*sin(theta);
		point.y = sign*(r-r*cos(theta));
		points[i] = point;
	}

	path_points = points;
}
*/

std::vector<cart_pose> tentacle::make_path()
{
	double s = r*theta;
	int n = round(s/grid_size);
	double delta_theta = fabs(theta/n);
	path_points = std::vector<cart_point>(n);
	path_poses = std::vector<cart_pose>(n);
	for(int i=0; i<n; i++)
	{
		double theta = delta_theta*i;
		cart_point point;
		point.x = r*sin(theta);
		point.y = sign*(r-r*cos(theta));
		path_points[i] = point;
		//print_cart_point_endline(point);
		//std::cout << sin(1.0) << "\n";
		cart_pose pose;
		pose.point = point;
		pose.theta = sign*theta;
		path_poses[i] = pose;
	}
	return path_poses;
}

std::vector<grid_cart_point> tentacle::make_grid_collision_box()
{
	std::vector<cart_point> collision_points;
	collision_points = make_collision_box();
	std::vector<grid_cart_point> tmp_grid = std::vector<grid_cart_point>(collision_points.size());
	for(int i=0; i<tmp_grid.size(); i++)
	{
		tmp_grid[i] = cart_to_grid(collision_points[i]);
	}
	std::vector<grid_cart_point> unique_grid = std::vector<grid_cart_point>(tmp_grid.size());
	int unique_count = 0;
	if(!tmp_grid.empty() && tmp_grid.size() != 0)
	{
		unique_count++;
		grid_cart_point default_point;
		default_point.x = 0;
		default_point.y = 0;
		unique_grid[0] = default_point;
	}
	for(int i=0; i<tmp_grid.size(); i++)
	{
		grid_cart_point current_point = tmp_grid[i];
		bool is_unique = true;
		for(int j=0; j<unique_count; j++)
		{
			if(grid_cart_points_equal(current_point, unique_grid[j]))
			{
				is_unique = false;
				break;
			}
		}
		if(is_unique)
		{
			unique_grid[unique_count] = current_point;
			unique_count++;
		}
	}

	std::vector<grid_cart_point> grid_collision_points = std::vector<grid_cart_point>(unique_count);

	for(int i=0; i<unique_count; i++)
	{
		grid_collision_points[i] = unique_grid[i];
	}
	//grid_collision_points = tmp_grid;
	return grid_collision_points;
}

bool tentacle::is_safe(std::vector<grid_cart_point> collision_points)
{
	for(int i=0; i<collision_points.size(); i++)
	{
		if(check_lookup(collision_points[i]))
			return false;
	}

	return true;
}

void tentacle::print_grid()
{
	std::vector<grid_cart_point> grid_collision_points = make_grid_collision_box();
	for(int i=0; i<grid_collision_points.size(); i++)
	{
		print_grid_cart_point_endline(grid_collision_points[i]);
	}
}

void tentacle::print_grid_inline(int index)
{
	std::vector<grid_cart_point> grid_collision_points = get_grid_from_lookup();
	print_grid_cart_point_inline(grid_collision_points[index]);
}

void tentacle::print_collision()
{
	std::vector<cart_point> collision_points = make_collision_box();
	for(int i=0; i<collision_points.size(); i++)
	{
		print_cart_point_endline(collision_points[i]);
	}
}

void tentacle::print_path()
{
	print_cart_points(path_points);
}

void tentacle::make_lookup_grid()
{
	std::vector<grid_cart_point> grid = make_grid_collision_box();
	int min_x, max_x, min_y, max_y;
	min_x = 0;
	max_x = 0;
	min_y = 0;
	max_y = 0;
	for(int i=0; i< grid.size(); i++)
	{
		if(grid[i].x < min_x)
		{
			min_x = grid[i].x;
		}
		else if(grid[i].x > max_x)
		{
			max_x = grid[i].x;
		}

		if(grid[i].y < min_y)
		{
			min_y = grid[i].y;
		}
		else if(grid[i].y > max_y)
		{
			max_y = grid[i].y;
		}
	}
	int grid_x = max_x + 1;
	int grid_y = max_y - min_y + 1;
	lookup_offset_x = 0;
	lookup_offset_y = min_y;
	look_up_grid = std::vector<std::vector<int> >(grid_x, std::vector<int>(grid_y));
	for(int i=0; i<grid.size(); i++)
	{
		add_to_lookup_grid(grid[i]);
	}
	return;
}

void tentacle::add_to_lookup_grid(grid_cart_point point)
{
	int x,y;
	x = point.x - lookup_offset_x;
	y = point.y - lookup_offset_y;

	if(look_up_grid.empty() || x < 0 || y < 0 || x >= look_up_grid.size() || y >= look_up_grid[x].size() )
		return;

	look_up_grid[x][y] = 1;
}

void tentacle::print_lookup_grid()
{
	for(int i=0; i<look_up_grid.size(); i++)
	{
		for(int j=0; j<look_up_grid[i].size(); j++)
		{
			if(j < look_up_grid[i].size() -1)
				std::cout << look_up_grid[i][j] << ",";
			else
				std::cout << look_up_grid[i][j] << "\n";
		}

	}
}

bool tentacle::check_lookup(grid_cart_point point)
{
	int x,y;
	x = point.x - lookup_offset_x;
	y = point.y - lookup_offset_y;

	if(look_up_grid.empty() || x < 0 || y < 0 || x >= look_up_grid.size() || y >= look_up_grid[x].size() )
		return false;

	return look_up_grid[x][y]==1;
}

std::vector<grid_cart_point> tentacle::get_grid_from_lookup()
{
	std::vector<grid_cart_point> grid_points =
		std::vector<grid_cart_point>(look_up_grid.size()*look_up_grid[0].size());
	int point_count = 0;
	for(int i=0; i<look_up_grid.size(); i++)
	{
		for(int j=0; j<look_up_grid[0].size(); j++)
		{
			if(look_up_grid[i][j] == 1)
			{
				grid_cart_point point;
				point.x = i + lookup_offset_x;
				point.y = j + lookup_offset_y;
				grid_points[point_count] = point;
				point_count++;
			}
		}
	}
	std::vector<grid_cart_point> short_grid_points = std::vector<grid_cart_point>(point_count);
	for(int i=0; i<point_count; i++)
	{
		short_grid_points[i] = grid_points[i];
	}

	return short_grid_points;
}

double tentacle::straight_line_dist(cart_point c1, cart_point c2)
{
    return sqrt(pow(c1.x-c2.x,2)+pow(c1.y-c2.y,2));
}

double tentacle::set_score(std::vector<cart_point> robot_path)
{
	long double sum = 0;
    /*int most_points = robot_path.size();
    if(most_points < path_points.size())
        most_points = path_points.size();
	double num_calcs = most_points;*/
	long int num_calcs = 0;
    int grid_ratio = 1;
    if(robot_path.size() != 0)
        grid_ratio = round(stl(robot_path[0], robot_path[1])/grid_size);

	if(grid_ratio == 0)
		grid_ratio++;

	for(int i=0; i<path_points.size(); i++)
	{
		for(int j=grid_ratio*i; j<grid_ratio*(i+1); j++)
		{
		    cart_point rp,pp;
		    if(path_points.size() < j)
		        pp = *(--path_points.end());
		    else
		        pp = path_points[j];

			if(robot_path.size() < i)
			    rp = robot_path[i];
			else
				rp = *(--robot_path.end());

		    sum += stl(rp,pp);
			num_calcs++;
		}
	}
	score = sum/(path_points.size()*LENG_CONST);//num_calcs;
	//ROS_INFO("grid_ratio %d", grid_ratio);
	return score;
}

void tentacle::print_grid(std::vector<grid_cart_point> obstacle_points)
{
	std::vector<grid_cart_point> grid_points = get_grid_from_lookup();
	for(int i=0; i<grid_points.size(); i++)
	{
		print_grid_cart_point_inline(grid_points[i]);
		if(i >= obstacle_points.size())
			print_grid_cart_point_endline(*(--obstacle_points.end()));
		else
			print_grid_cart_point_endline(obstacle_points[i]);
	}
}

void tentacle::print_grid(std::vector<grid_cart_point> obstacle_points, std::vector<cart_point> target_path)
{
	std::vector<grid_cart_point> grid_points = get_grid_from_lookup();
	/*for(int i=0; i<grid_points.size(); i++)
	{
		print_grid_cart_point_inline(grid_points[i]);
		if(i >= target_path.size())
			print_grid_cart_point_inline(cart_to_grid(*(--target_path.end())));
		else
			print_grid_cart_point_inline(cart_to_grid(target_path[i]));

		if(i >= obstacle_points.size())
			print_grid_cart_point_endline(*(--obstacle_points.end()));
		else
			print_grid_cart_point_endline(obstacle_points[i]);
	}*/
	int max_num = grid_points.size();
	if(max_num < obstacle_points.size())
		max_num = obstacle_points.size();
	if(max_num < target_path.size())
		max_num = target_path.size();

	ROS_INFO("MAX is %d, of %d %d %d", max_num, (int)grid_points.size(), (int)obstacle_points.size(), (int)target_path.size());

	for(int i=0; i<max_num; i++)
	{
		if(i >= grid_points.size())
			print_grid_cart_point_inline(*(--grid_points.end()));
		else
			print_grid_cart_point_inline(grid_points[i]);
		if(i >= target_path.size())
			print_grid_cart_point_inline(cart_to_grid(*(--target_path.end())));
		else
			print_grid_cart_point_inline(cart_to_grid(target_path[i]));

		if(i >= obstacle_points.size())
			print_grid_cart_point_endline(*(--obstacle_points.end()));
		else
			print_grid_cart_point_endline(obstacle_points[i]);
	}
}

void tentacle::print_path(std::vector<grid_cart_point> obstacle_points, std::vector<cart_point> target_path)
{
	int max_num = path_points.size();
	if(max_num < obstacle_points.size())
		max_num = obstacle_points.size();
	if(max_num < target_path.size())
		max_num = target_path.size();

	//ROS_INFO("MAX is %d, of %d %d %d", max_num, (int)path_points.size(), (int)obstacle_points.size(), (int)target_path.size());

	for(int i=0; i<max_num; i++)
	{
		if(i >= path_points.size())
			print_grid_cart_point_inline(cart_to_grid(*(--path_points.end())));
		else
			print_grid_cart_point_inline(cart_to_grid(path_points[i]));
		if(i >= obstacle_points.size())
			print_grid_cart_point_inline(*(--obstacle_points.end()));
		else
			print_grid_cart_point_inline(obstacle_points[i]);

		if(i >= target_path.size())
			print_grid_cart_point_endline(cart_to_grid(*(--target_path.end())));
		else
			print_grid_cart_point_endline(cart_to_grid(target_path[i]));
	}
}

double tentacle::get_v()
{
	return v;
}

double tentacle::get_w()
{
	return w;
}

void tentacle::set_cost_score(std::vector<cost_grid_point> cost_grid)
{
    //ROS_INFO("using cost grid");
    cost_score = 0;
    for(int i=0; i<cost_grid.size(); i++)
    {
        //print_grid_cart_point(cost_grid[i].point);
        if(check_lookup(cost_grid[i].point))
        {
            cost_score += cost_grid[i].cost;
            //ROS_INFO("cost = %f", cost_grid[i].cost);
        }
    }
}

void tentacle::update_combined_score()
{
    if(score < 0 || cost_score < 0)
    {
        combined_score = BIG_NUM;
    }
    combined_score = PATH_WEIGHT*score+COST_WEIGHT*cost_score;
}

double tentacle::get_cost_score()
{
    return cost_score;
}

double tentacle::get_combined_score()
{
    update_combined_score();
    return combined_score;
}

void tentacle::update_scores(std::vector<cart_point> robot_path, std::vector<cost_grid_point> cost_grid)
{
    set_score(robot_path);
    set_cost_score(cost_grid);
    update_combined_score();
    //ROS_INFO("combined_score=%f", cost_score);
}

double tentacle::get_score()
{
    return score;
}
void tentacle::update_scores(std::vector<cart_point> robot_path,
        cost_mapper mapper)
{
    cost_score = mapper.score_path(path_points);
    set_score(robot_path);
    update_combined_score();
}


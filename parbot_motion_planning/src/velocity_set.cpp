#include "velocity_set.h"

velocity_set::velocity_set(double _v, std::vector<double> thetas, robot_footprint* robot_foot, double _width,
 double _grid_size)
{
	v = _v;
	width = _width;
	grid_size = _grid_size;
	safe_tentacles = std::vector<tentacle*>(0);
	make_tentacles(thetas, robot_foot);
};

void velocity_set::make_tentacles(std::vector<double> thetas,
	robot_footprint* robot_foot)
{
	tentacles = std::vector<tentacle*>(thetas.size());
	for(int i=0; i<tentacles.size(); i++)
	{
		tentacles[i] = new tentacle(v,thetas[i],robot_foot);
		//ROS_INFO("Completed tentacle %d", i+1);
	}
}

void velocity_set::print_grid()
{
	int most_points = tentacles[0]->make_grid_collision_box().size();
	for(int i=1; i<tentacles.size(); i++)
	{
		int current_points = tentacles[i]->make_grid_collision_box().size();
		if(current_points > most_points)
			most_points = current_points;
	}
	std::vector<std::vector<grid_cart_point> > points =
		std::vector<std::vector<grid_cart_point> >(tentacles.size(), std::vector<grid_cart_point>( most_points));
	for(int i=0; i<tentacles.size(); i++)
	{
		points[i] = tentacles[i]->make_grid_collision_box();
	}

	for(int i=0; i< most_points; i++)
	{
		for(int j=0; j<points.size()-1; j++)
		{
			if(i >= points[j].size())
				print_grid_cart_point_inline(*(--points[j].end()));
			else
				print_grid_cart_point_inline(points[j][i]);
		}
		if(i >= points[points.size()-1].size())
			print_grid_cart_point_endline(*(--points[points.size()-1].end()));
		else
			print_grid_cart_point_endline(points[points.size()-1][i]);
	}
}

std::vector<tentacle*> velocity_set::make_safe_tentacles(std::vector<grid_cart_point> scan_points)
{
	std::vector<tentacle*> tmp_tents = std::vector<tentacle*>(tentacles.size());
	int safe_count = 0;

	for(int i=0; i<tentacles.size(); i++)
	{
		if(tentacles[i]->is_safe(scan_points))
		{
			tmp_tents[safe_count] = tentacles[i];
			safe_count++;
		}
	}

	std::vector<tentacle*> safe_tentacles = std::vector<tentacle*>(safe_count);

	for(int i=0; i<safe_count; i++)
	{
		safe_tentacles[i] = tmp_tents[i];
	}

	return safe_tentacles;
}

void velocity_set::print_path()
{
	int num_areas = tentacles.size();
	for(int i=0; i<tentacles[0]->path_points.size(); i++)
	{
		for(int j=0; j<tentacles.size(); j++)
		{
			if(j < tentacles.size() - 1)
				print_cart_point_inline(tentacles[j]->path_points[i]);
			else
				print_cart_point_endline(tentacles[j]->path_points[i]);
		}
	}
}

void velocity_set::print_path_inline(int index)
{
		for(int j=0; j<tentacles.size(); j++)
		{
			if(index >= tentacles[j]->path_points.size())
			{
				print_cart_point_inline(*(--tentacles[j]->path_points.end()));
			}
			else
			{
				print_cart_point_inline(tentacles[j]->path_points[index]);
			}
		}
}


int velocity_set::len_grid_points()
{
	int num_areas = tentacles.size();
	int num_points = tentacles[0]->make_grid_collision_box().size();
	std::vector<std::vector<grid_cart_point> > tentacle_grid_collision_box =
		std::vector<std::vector<grid_cart_point> >(num_areas, std::vector<grid_cart_point>(num_points));
	for(int i=0; i<tentacles.size(); i++)
	{
		tentacle_grid_collision_box[i] = tentacles[i]->get_grid_from_lookup();
	}
	for(int i=0; i<tentacles.size(); i++)
	{
		if(num_points < tentacle_grid_collision_box[i].size())
		{
			num_points = tentacle_grid_collision_box[i].size();
		}
	}
	return num_points;
}

int velocity_set::len_grid_points(std::vector<tentacle*> tents)
{
	int num_areas = tents.size();
	int num_points = tents[0]->get_grid_from_lookup().size();
	std::vector<std::vector<grid_cart_point> > tentacle_grid_collision_box =
		std::vector<std::vector<grid_cart_point> >(num_areas, std::vector<grid_cart_point>(num_points));
	for(int i=0; i<tents.size(); i++)
	{
		tentacle_grid_collision_box[i] = tents[i]->get_grid_from_lookup();
	}
	for(int i=0; i<tents.size(); i++)
	{
		if(num_points < tentacle_grid_collision_box[i].size())
		{
			num_points = tentacle_grid_collision_box[i].size();
		}
	}
	return num_points;
}

void velocity_set::update_safe_tentacles(std::vector<grid_cart_point> scan_point)
{
	safe_tentacles = make_safe_tentacles(scan_point);
}

void velocity_set::print_safe_paths()
{
	if(safe_tentacles.empty())
		return;
	int num_areas = safe_tentacles.size();
	for(int i=0; i<safe_tentacles[0]->path_points.size(); i++)
	{
		for(int j=0; j<safe_tentacles.size(); j++)
		{
			if(j < safe_tentacles.size() - 1)
				print_grid_cart_point_inline(cart_to_grid(safe_tentacles[j]->path_points[i]));
			else
				print_grid_cart_point_endline(cart_to_grid(safe_tentacles[j]->path_points[i]));
		}
	}
}

void velocity_set::print_safe_grid()
{
	print_grid(safe_tentacles);
}

void velocity_set::print_safe_grid(std::vector<grid_cart_point> collision_points)
{
	int most_points = safe_tentacles[0]->get_grid_from_lookup().size();
	for(int i=1; i<safe_tentacles.size(); i++)
	{
		int current_points = safe_tentacles[i]->get_grid_from_lookup().size();
		if(current_points > most_points)
			most_points = current_points;
	}
	std::vector<std::vector<grid_cart_point> > points =
		std::vector<std::vector<grid_cart_point> >(safe_tentacles.size(),
		std::vector<grid_cart_point>( most_points));
	for(int i=0; i<safe_tentacles.size(); i++)
	{
		points[i] = safe_tentacles[i]->get_grid_from_lookup();
	}

	for(int i=0; i< most_points; i++)
	{
		for(int j=0; j<points.size(); j++)
		{
			if(i >= points[j].size())
				print_grid_cart_point_inline(*(--points[j].end()));
			else
				print_grid_cart_point_inline(points[j][i]);
		}
		if(i >= collision_points.size())
			print_grid_cart_point_endline(*(--collision_points.end()));
		else
			print_grid_cart_point_endline(collision_points[i]);
	}
}

std::vector<std::vector<grid_cart_point> > velocity_set::get_collision_grids()
{
	std::vector<std::vector<grid_cart_point> > grids =
		std::vector<std::vector<grid_cart_point> >(tentacles.size(), std::vector<grid_cart_point>(len_grid_points()));

	for(int i=0; i<tentacles.size(); i++)
	{
		grids[i] = tentacles[i]->get_grid_from_lookup();//make_grid_collision_box();
	}

	return grids;
}

void velocity_set::print_grid(std::vector<tentacle*> tents)
{
	int most_points = tents[0]->get_grid_from_lookup().size();
	for(int i=1; i<tents.size(); i++)
	{
		int current_points = tents[i]->get_grid_from_lookup().size();
		if(current_points > most_points)
			most_points = current_points;
	}
	std::vector<std::vector<grid_cart_point> > points =
		std::vector<std::vector<grid_cart_point> >(tents.size(), std::vector<grid_cart_point>( most_points));
	for(int i=0; i<tents.size(); i++)
	{
		points[i] = tents[i]->get_grid_from_lookup();
	}

	for(int i=0; i< most_points; i++)
	{
		for(int j=0; j<points.size()-1; j++)
		{
			if(i >= points[j].size())
				print_grid_cart_point_inline(*(--points[j].end()));
			else
				print_grid_cart_point_inline(points[j][i]);
		}
		if(i >= points[points.size()-1].size())
			print_grid_cart_point_endline(*(--points[points.size()-1].end()));
		else
			print_grid_cart_point_endline(points[points.size()-1][i]);
	}
}

std::vector<tentacle*> velocity_set::get_safe_tentacles()
{
	return safe_tentacles;
}

bool velocity_set::has_safe_tentacle(std::vector<grid_cart_point> obstacle_points)
{
	for(int i=0; i<tentacles.size(); i++)
	{
		if(tentacles[i]->is_safe(obstacle_points))
			return true;
	}
	return false;
}

void velocity_set::print_safe_grid_inline(int index)
{
	for(int i=0; i<safe_tentacles.size(); i++)
	{
		if(safe_tentacles[i]->get_grid_from_lookup().size() <= index)
			print_grid_cart_point_inline(*(--safe_tentacles[i]->get_grid_from_lookup().end()));
		else
			print_grid_cart_point_inline(safe_tentacles[i]->get_grid_from_lookup()[index]);
	}
}

void velocity_set::print_safe_paths_inline(int index)
{
	if(safe_tentacles.empty())
		return;
	int num_areas = safe_tentacles.size();

	for(int j=0; j<safe_tentacles.size(); j++)
	{
		if(index >= safe_tentacles[j]->path_points.size())
			print_grid_cart_point_inline(cart_to_grid(*(--safe_tentacles[j]->path_points.end())));
		else
			print_grid_cart_point_inline(cart_to_grid(safe_tentacles[j]->path_points[index]));
	}
}

int velocity_set::len_path_points()
{
	int num_points = 0;
	for(int i=0; i<tentacles.size(); i++)
	{
		if(num_points < tentacles[i]->path_points.size())
		{
			num_points = tentacles[i]->path_points.size();
		}
	}
	return num_points;
}

void velocity_set::print_paths_grid_inline(int index)
{
		for(int j=0; j<tentacles.size(); j++)
		{
			if(index >= tentacles[j]->path_points.size())
			{
				print_grid_cart_point_inline(cart_to_grid(*(--tentacles[j]->path_points.end())));
			}
			else
			{
				print_grid_cart_point_inline(cart_to_grid(tentacles[j]->path_points[index]));
			}
		}
}

void velocity_set::print_paths_grid_endline(int index)
{
		for(int j=0; j<tentacles.size(); j++)
		{
			grid_cart_point point;
			if(index >= tentacles[j]->path_points.size())
			{
				point = cart_to_grid(*(--tentacles[j]->path_points.end()));
			}
			else
			{
				point = cart_to_grid(tentacles[j]->path_points[index]);
			}

			if(j < tentacles.size() - 1)
				print_grid_cart_point_inline(point);
			else
				print_grid_cart_point_endline(point);
		}
}

tentacle* velocity_set::lowest_safe_score(std::vector<cart_point> robot_path)
{
	//ROS_INFO("testing set");
	if(safe_tentacles.empty())
	{
		//ROS_INFO("lowest score something is wrong");
		return NULL;
	}
	else if(safe_tentacles.size() == 0)
	{
		//ROS_INFO("none are safe, or something is wrong");
		return NULL;
	}
	//ROS_INFO("set has options");
	safe_tentacles[0]->set_score(robot_path);
	tentacle* lowest_tent = safe_tentacles[0];
	//ROS_INFO("set index 0 as best, testing with no more checks");
	for(int i=1; i<safe_tentacles.size(); i++)
	{
		if(lowest_tent->score > safe_tentacles[i]->set_score(robot_path))
			lowest_tent = safe_tentacles[i];
       // ROS_INFO("v=%f w=%f score %f", safe_tentacles[i]->get_v(), safe_tentacles[i]->get_w(), safe_tentacles[i]->score);
	}
	return lowest_tent;
}

tentacle* velocity_set::lowest_safe_score(std::vector<cart_point> robot_path, std::vector<cost_grid_point> cost_map)
{
    //ROS_INFO("testing set");
    if(safe_tentacles.empty() || safe_tentacles.size() == 0)
    {
        //ROS_INFO("none are safe, or something is wrong");
        return NULL;
    }
    //ROS_INFO("set has options");
    safe_tentacles[0]->update_scores(robot_path, cost_map);
    tentacle* lowest_tent = safe_tentacles[0];
    //ROS_INFO("set index 0 as best, testing with no more checks");
    for(int i=1; i<safe_tentacles.size(); i++)
    {
        safe_tentacles[i]->update_scores(robot_path, cost_map);
        if(lowest_tent->get_combined_score() > safe_tentacles[i]->get_combined_score())
            lowest_tent = safe_tentacles[i];
       // ROS_INFO("v=%f w=%f score %f", safe_tentacles[i]->get_v(), safe_tentacles[i]->get_w(), safe_tentacles[i]->score);
    }
    return lowest_tent;
}

tentacle* velocity_set::lowest_safe_score(std::vector<cart_point> robot_path,
    cost_mapper mapper)
{
    //ROS_INFO("testing set");
    if(safe_tentacles.empty() || safe_tentacles.size() == 0)
    {
        //ROS_INFO("none are safe, or something is wrong");
        return NULL;
    }
    //ROS_INFO("set has options");
    safe_tentacles[0]->update_scores(robot_path, mapper);
    tentacle* lowest_tent = safe_tentacles[0];
    //ROS_INFO("set index 0 as best, testing with no more checks");
    for(int i=1; i<safe_tentacles.size(); i++)
    {
        safe_tentacles[i]->update_scores(robot_path, mapper);
        if(lowest_tent->get_combined_score() > safe_tentacles[i]->get_combined_score())
            lowest_tent = safe_tentacles[i];
       // ROS_INFO("v=%f w=%f score %f", safe_tentacles[i]->get_v(), safe_tentacles[i]->get_w(), safe_tentacles[i]->score);
    }
    return lowest_tent;
}

std::vector<tentacle*> velocity_set::lowest_safe_score(std::vector<cart_point> robot_path, int num_to_find)
{
    //ROS_INFO("Starting search in vel set");
    if(safe_tentacles.empty() || safe_tentacles.size() == 0)
    {
        //ROS_INFO("none are safe, or something is wrong");
        std::vector<tentacle*> crap = std::vector<tentacle*>(0);
        return crap;
    }

    if(num_to_find > safe_tentacles.size())
        num_to_find = safe_tentacles.size();

    //ROS_INFO("looking for %d tents",num_to_find);

    std::vector<tentacle*> best_tents = std::vector<tentacle*>(num_to_find);
    safe_tentacles[0]->set_score(robot_path);
    tentacle* worst_of_best = safe_tentacles[0];
    best_tents[0] = worst_of_best;

    //ROS_INFO("Created one of them");

    for(int i=1; i<best_tents.size(); i++)
    {
        safe_tentacles[i]->set_score(robot_path);
        best_tents[i] = safe_tentacles[i];
        if(best_tents[i]->score > worst_of_best->score)
            worst_of_best = best_tents[i];
    }

    //ROS_INFO("got first %d", num_to_find);

    if(best_tents.size() == safe_tentacles.size())
    {
        //ROS_INFO("No more tents sending back %d", (int)best_tents.size());
        return best_tents;
    }

    for(int i=best_tents.size(); i<safe_tentacles.size(); i++)
    {
        //ROS_INFO("working on tent %d of %d",i, (int)safe_tentacles.size());
        safe_tentacles[i]->set_score(robot_path);
        //ROS_INFO("set_score");
        if(safe_tentacles[i]->score < worst_of_best->score)
        {
            //ROS_INFO("Better than worst: %f < %f", safe_tentacles[i]->score, worst_of_best->score);
            tentacle* new_worst = safe_tentacles[i];
            //ROS_INFO("made new worst");
            for(int j=0; j<best_tents.size(); j++)
            {
                //ROS_INFO("looking for old worst at %d",j);
               // ROS_INFO("scores %f ", best_tents[j]->score);//, worst_of_best->score);
                if(best_tents[j]->score == worst_of_best->score)
                {
                   //ROS_INFO("found old worst");
                    best_tents[j] = new_worst;
                    worst_of_best = best_tents[j];
                    break;
                }
            }

            for(int j=0; j<best_tents.size(); j++)
            {
                if(best_tents[j]->score > worst_of_best->score)
                    worst_of_best = best_tents[j];
            }
        }
    }
    return best_tents;
}


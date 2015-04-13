#include "Square_Grid_Footprint.h"

Square_Grid_Footprint::Square_Grid_Footprint(double _target_grid_size, double _source_grid_size)
{
    source_grid_size = _source_grid_size;
    target_grid_size = _target_grid_size;
    make_footprint();
};

void Square_Grid_Footprint::make_footprint()
{
    int grid_ratio = round(source_grid_size/target_grid_size);
    if(grid_ratio <= 1)
    {
        grid_ratio = 1;
        foot_print = std::vector<cart_point>(1);
        cart_point point;
        point.x = 0;
        point.y = 0;
        foot_print[0] = point;
    }
    else
    {
        foot_print = std::vector<cart_point>(pow(grid_ratio,2));
        for(int y=0; y<grid_ratio; y++)
        {
            for(int x=0; x<grid_ratio; x++)
            {
                double center = target_grid_size*round(grid_ratio/2);
                cart_point point;
                point.x = x*target_grid_size - center;
                point.y = y*target_grid_size - center;
                foot_print[y*grid_ratio+x] = point;
            }
        }
    }
}

std::vector<cart_point> Square_Grid_Footprint::points_from_point(cart_point point, double _source_grid_size)
{
    std::vector<cart_point> output_fp = std::vector<cart_point>(foot_print.size());
    for(int i=0; i<output_fp.size(); i++)
    {
        output_fp[i] = move_point(foot_print[i], point);
    }
    return output_fp;
}

cart_point Square_Grid_Footprint::move_point(cart_point local_point, cart_point target_point)
{
    cart_point new_point;
    new_point.x = local_point.x + target_point.x;
    new_point.y = local_point.y + target_point.y;
    return new_point;
}

double Square_Grid_Footprint::get_target_res()
{
    return target_grid_size;
}

double Square_Grid_Footprint::get_source_res()
{
    return source_grid_size;
}

std::vector<cart_point> Square_Grid_Footprint::points_from_point(cart_point point)
{
    points_from_point(point, target_grid_size);
}

std::vector<std::vector<cart_point> > Square_Grid_Footprint::points_from_points(std::vector<cart_point> points)
{
    std::vector<std::vector<cart_point> > all_points = std::vector<std::vector<cart_point> >(points.size(), std::vector<cart_point>(0));
    for(int i=0; i<all_points.size(); i++)
    {
        all_points[i] = points_from_point(points[i]);
    }
    return all_points;
}

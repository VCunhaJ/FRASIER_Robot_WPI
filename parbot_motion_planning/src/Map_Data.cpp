#include "Map_Data.h"

Map_Data::Map_Data(std::vector<grid_cart_point> _points, int _x_dimension, int _y_dimension, double _grid_size)
{
    init(_points, _x_dimension, _y_dimension, _grid_size);
};

Map_Data::Map_Data(std::vector<cart_point> _points, int _x_dimension, int _y_dimension,  double _grid_size)
{
    init(cart_to_grid(_points), _x_dimension, _y_dimension, _grid_size);
};

void Map_Data::init(std::vector<grid_cart_point> _points, int _x_dimension, int _y_dimension, double _grid_size)
{
    points = _points;
    grid_size = _grid_size;
    x_dimension = _x_dimension;
    y_dimension = _y_dimension;
}

std::vector<grid_cart_point> Map_Data::get_points()
{
    return points;
}

double Map_Data::get_grid_size()
{
    return grid_size;
}

int Map_Data::get_x_dimension()
{
    return x_dimension;
}

int Map_Data::get_y_dimension()
{
    return y_dimension;
}

void Map_Data::set_occ_grid(std::vector<std::vector<double> > ocg)
{
    occupancy_grid = ocg;
}

double Map_Data::value_at(int x, int y)
{
    return occupancy_grid[x][y];
}

std::vector<std::vector<double> > Map_Data::get_occ_grid()
{
    return occupancy_grid;
}

#ifndef MAP_DATA_H
#define MAP_DATA_H
#include "motion_planning_structures.h"

class Map_Data{

public:
    Map_Data(std::vector<grid_cart_point> _points,
         int _x_dimension, int _y_dimension, double _grid_size);
    Map_Data(std::vector<cart_point> _points, int _x_dimension,
        int _y_dimension,  double _grid_size=GRID_SIZE);

    void set_occ_grid(std::vector<std::vector<double> > ocg);
    double value_at(int x, int y);
    std::vector<std::vector<double> > get_occ_grid();

    std::vector<grid_cart_point> get_points();
    double get_grid_size();
    int get_x_dimension();
    int get_y_dimension();

private:
    std::vector<grid_cart_point> points;
    double grid_size;
    int x_dimension, y_dimension;
    std::vector<std::vector<double> > occupancy_grid;

    void init(std::vector<grid_cart_point> _points, int _x_dimension,
     int _y_dimension, double _grid_size);
};
#endif

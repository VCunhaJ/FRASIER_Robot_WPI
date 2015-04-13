#include <vector>
#define THRESHOLD 0 //determines if cell is occupied before creating node

std::vector convert_occupancy_grid(nav_msgs::OccupancyGrid grid)
{
    int obstacles = 0; // number of nodes in this iteration of y
    float res = grid.info.resolution; //meters per cell
    float width = grid.info.width;
    float height = grid.info.height;
    std::vector<cost_point> all_obstacles= std::vector<cost_point>(width*height);

    for (int y = 0; y < height; ++y)
    {
        //cout<<"y= " << y <<"\n";
        int len_prev_row = prev_row.size(); //number of nodes in previous row
        for (int x = 0; x < width; ++x)
        {
            int current_data = ((y*width) + x); //may need to cast to int
            if ((grid.data[current_data] > THRESHOLD) && (grid.data[current_data] != -1))
            {
                cart_point point;
                point.x = (res*x - (res*width/2));
                point.y = (res*y - (res*height/2));

                cost_point c_point;
                c_point.point=point;
                c_point.cost=grid.data[current_data];

                all_obstacles[obstacles] = c_point;
                obstacles++; //increase number o  in this row
            }
        }
    }
    all_obstacles.resize(obstacles);
    return all_obstacles;
}

#include "RecoveryTentacle.h"

RecoveryTentacle::RecoveryTentacle(double v, double theta, robot_footprint* robo_foot)
{
    scoreTent = new tentacle(v, theta, robo_foot);
    motionTent = new tentacle(-1*v, theta, robo_foot);
};

double RecoveryTentacle::set_score(std::vector<cart_point> robot_path)
{
    return scoreTent->set_score(robot_path);
}

bool RecoveryTentacle::isSafe(std::vector<grid_cart_point> collision_points)
{
    return motionTent->is_safe(collision_points);
}

double RecoveryTentacle::get_v()
{
    return motionTent->get_v();
}

double RecoveryTentacle::get_w()
{
    return motionTent->get_w();
}

double RecoveryTentacle::get_score()
{
    return scoreTent->score;
}

tentacle* RecoveryTentacle::get_tent()
{
    return motionTent;
}

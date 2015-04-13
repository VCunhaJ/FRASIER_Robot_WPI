#ifndef RECOVERYTENT
#define RECOVERYTENT
#include "tentacle.h"
#include "path_handler.h"

class RecoveryTentacle
{
public:
	RecoveryTentacle(double v, double theta, robot_footprint* robo_foot);

	double get_score(); //scoreTent->score
	double set_score(std::vector<cart_point> robot_path); //scoreTent->double set_score(robot_path)
	bool isSafe(std::vector<grid_cart_point> collision_points); //motionTent->is_safe(std::vector<grid_cart_point> collision_points)
	double get_v(); //motionTent->get_v();
	double get_w(); //motionTent->get_w();
	tentacle* get_tent(); //motionTent
private:
	tentacle* scoreTent;
    tentacle* motionTent;//score is forward, motion is backwards
};

#endif

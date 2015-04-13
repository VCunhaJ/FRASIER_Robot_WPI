#ifndef RECOVERYSET
#define RECOVERYSET
#include "RecoveryTentacle.h"


class RecoverySet
{
public:
	RecoverySet(double v, std::vector<double> thetas, robot_footprint* robo_foot, path_handler* _pah);

	tentacle* recover(std::vector<grid_cart_point> collision_points);
private:
	std::vector<RecoveryTentacle*> tents, safe_tents;
	path_handler* pah;

	void setSafe(std::vector<grid_cart_point> collision_points);
	tentacle* getBestTent();
};
#endif

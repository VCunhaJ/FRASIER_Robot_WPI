#ifndef RECOVERY
#define RECOVERY
#include "RecoverySet.h"

class Recovery
{
public:
	Recovery(double v, std::vector<double> thetas, robot_footprint* robo_foot, path_handler* _pah);

	tentacle* recover(std::vector<grid_cart_point> collision_points);
	bool canRecover();
	tentacle* getBest();
	void clear();
private:
	std::vector<RecoverySet*> sets;
	bool canDo, inRecovery;
	tentacle* bestTent;
    ros::Time recoveryStartTime;
};

#endif

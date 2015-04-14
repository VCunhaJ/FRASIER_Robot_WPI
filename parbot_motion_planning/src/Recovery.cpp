#include "Recovery.h"

#define ENABLED false

Recovery::Recovery(double v, std::vector<double> thetas, robot_footprint* robo_foot, path_handler* _pah)
{
    sets = std::vector<RecoverySet*>(1);
    sets[0] = new RecoverySet(v, thetas, robo_foot, _pah);
    canDo = false;
    inRecovery = false;
    bestTent = NULL;
};

bool Recovery::canRecover()
{
    return canDo;
}

tentacle* Recovery::getBest()
{
    return bestTent;
}

void Recovery::clear()
{
    inRecovery = false;
}

tentacle* Recovery::recover(std::vector<grid_cart_point> collision_points)
{
    if(!ENABLED)
    {
        return NULL;
    }

    ros::Time currentTime = ros::Time::now();
    if(!inRecovery)
    {
        recoveryStartTime = ros::Time::now();
        canDo = true;
        inRecovery = true;
        return NULL;
    }


    double time_since_start = ((double)currentTime.sec-(double)recoveryStartTime.sec)+((double)currentTime.nsec-(double)recoveryStartTime.nsec)/1000000000;
    if(time_since_start < 1)
    {
        return NULL;
    }
    else
    {
        if(sets.size() < 1)
        {
            return NULL;
        }

        tentacle* best = sets[0]->recover(collision_points);

        for(int i=0; i<sets.size(); i++)
        {
            tentacle* currentTent = sets[i]->recover(collision_points);
            if(best == NULL)
            {
                best = currentTent;
            }
            else if(currentTent == NULL)
            {
                continue;
            }
            else if(currentTent->score < best->score)
            {
                best = currentTent;
            }
        }

        if(best == NULL)
        {
            canDo = false;
        }

        return best;
    }
}

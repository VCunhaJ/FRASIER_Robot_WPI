#include "RecoverySet.h"

RecoverySet::RecoverySet(double v, std::vector<double> thetas, robot_footprint* robo_foot, path_handler* _pah)
{
    tents = std::vector<RecoveryTentacle*>(thetas.size());
    safe_tents = std::vector<RecoveryTentacle*>(0);

    for(int i=0; i<tents.size(); i++)
    {
        tents[i] = new RecoveryTentacle(v, thetas[i], robo_foot);
    }

    pah = _pah;
};

void RecoverySet::setSafe(std::vector<grid_cart_point> collision_points)
{
    int num_safe = 0;
    safe_tents = std::vector<RecoveryTentacle*>(tents.size());
    for(int i=0; i<tents.size(); i++)
    {
        if(tents[i]->isSafe(collision_points))
        {
            safe_tents[num_safe] = tents[i];
            num_safe++;
        }
    }
    safe_tents.resize(num_safe);
}

tentacle* RecoverySet::getBestTent()
{
    if(safe_tents.size() == 0)
        return NULL;

    RecoveryTentacle* best = safe_tents[0];
    best->set_score(pah->robot_path);

    for(int i=1; i<safe_tents.size(); i++)
    {
        safe_tents[i]->set_score(pah->robot_path);
        if(safe_tents[i]->get_score() < best->get_score())
        {
            best = safe_tents[i];
        }
    }
    return best->get_tent();
}

tentacle* RecoverySet::recover(std::vector<grid_cart_point> collision_points)
{
    setSafe(collision_points);
    return getBestTent();
}

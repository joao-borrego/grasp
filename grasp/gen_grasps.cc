/*!
    \file grasp/gen_grasps.cc
    \brief Generate candidate grasps

    TODO

    \author João Borrego : jsbruglie
*/

#include "gen_grasps.hh"

void obtainGrasps(std::vector<Grasp> & grasps)
{
    // DEBUG
    for (int i = 0; i < 10; i++)
    {
        ignition::math::Pose3d pose(0.0,0.0,0.0,0,1.57,0);
        grasps.emplace_back(pose);
    }
}

//////////////////////////////////////////////////

// Random integer generator
std::random_device rng;
std::mt19937 mt_rng(rng());
std::uniform_int_distribution<int> uniform_dist;

//////////////////////////////////////////////////
double getRandomDouble(double min, double max)
{
    double aux = ((double) uniform_dist(mt_rng)) / (double) RAND_MAX;
    return aux * (max - min) + min;
}

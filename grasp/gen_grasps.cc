/*!
    \file grasp/gen_grasps.cc
    \brief Generate candidate grasps

    TODO

    \author João Borrego : jsbruglie
*/

#include "gen_grasps.hh"

void obtainGrasps(std::string file_name, std::vector<Grasp> & grasps)
{
    std::string robot("vizzy");
    double tmp[6];

    try
    {
        YAML::Node config = YAML::LoadFile(file_name);
        int num_grasps = config["object"]["grasp_candidates"][robot].size();
        for (int i = 0; i < num_grasps; i++)
        {
            YAML::Node pose_node =
                config["object"]["grasp_candidates"][robot][i]["pose"];
            for (int j = 0; j < 6; j++) { tmp[j] = pose_node[j].as<double>(); }
            ignition::math::Pose3d pose(tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5]);
            grasps.emplace_back(pose);
        }
    }
    catch (YAML::Exception& yamlException)
    {
        std::cerr << "Unable to parse " << file_name << "\n";
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

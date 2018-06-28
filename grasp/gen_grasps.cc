/*!
    \file grasp/gen_grasps.cc
    \brief Generate candidate grasps

    TODO

    \author João Borrego : jsbruglie
*/

#include "gen_grasps.hh"

using namespace ignition::math;

void obtainGrasps(std::string file_name, std::vector<Grasp> & grasps)
{
    std::string robot("vizzy");
    Matrix4d tf_mat (Matrix4d::Identity);

    try
    {
        YAML::Node config = YAML::LoadFile(file_name);
        int num_grasps = config["object"]["grasp_candidates"][robot].size();
        for (int i = 0; i < num_grasps; i++)
        {
            YAML::Node transform_mat_node =
                config["object"]["grasp_candidates"][robot][i]["tf"];
            for (int j = 0; j < 12; j++) {
                tf_mat(j/4,j%4) = transform_mat_node[j].as<double>();
            }
            grasps.emplace_back(tf_mat);
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

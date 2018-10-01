/*!
    \file grasp/Grasp.cc
    \brief Grasp representation

    \author João Borrego : jsbruglie
*/

#include "Grasp.hh"

//////////////////////////////////////////////////
Grasp::Grasp():
    t_gripper_object(ignition::math::Matrix4d::Identity),
    metric(0.0)
{
}

//////////////////////////////////////////////////
Grasp::Grasp(ignition::math::Matrix4d t_gripper_object_):
    t_gripper_object(t_gripper_object_),
    metric(0.0)
{
}

//////////////////////////////////////////////////
void Grasp::loadFromYml(
    const std::string & file_name,
    const std::string & robot,
    const std::string & object_name,
    std::vector<Grasp> & grasps)
{
    ignition::math::Matrix4d tf_matrix (ignition::math::Matrix4d::Identity);
    try
    {
        YAML::Node config = YAML::LoadFile(file_name);
        int num_grasps = config["object"]["grasp_candidates"][robot].size();
        for (int i = 0; i < num_grasps; i++)
        {
            YAML::Node transform_mat_node =
                config["object"]["grasp_candidates"][robot][i]["tf"];
            for (int j = 0; j < 12; j++) {
                tf_matrix(j/4,j%4) = transform_mat_node[j].as<double>();
            }
            grasps.emplace_back(tf_matrix);
        }
    }
    catch (YAML::Exception& yamlException)
    {
        errorPrintTrace("Unable to parse " << file_name);
    }
}

//////////////////////////////////////////////////
void Grasp::writeToYml(
    const std::string & file_name,
    const std::string & robot,
    const std::string & object_name,
    const std::vector<Grasp> & grasps)
{
    YAML::Node root;
    unsigned int size = grasps.size();
    for (int i = 0; i < size; i++)
    {
        std::string str_i = std::to_string(i);
        root["object"]["grasp_candidates"][robot][str_i]["success"] = 
            grasps.at(i).metric;
    }

    std::ofstream fout(file_name);
    if (!fout) {
        errorPrintTrace("Could not write to " << file_name);
    } else {
        fout << root;
        debugPrintTrace("All grasps written to " << file_name);
    }
}

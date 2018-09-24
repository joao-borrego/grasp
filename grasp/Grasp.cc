/*!
    \file src/Grasp.cc
    \brief Grasp representation

    \author João Borrego : jsbruglie
*/

#include "Grasp.hh"

//////////////////////////////////////////////////
Grasp::Grasp():
    t_gripper_object(ignition::math::Matrix4d::Identity)
{
}

//////////////////////////////////////////////////
Grasp::Grasp(ignition::math::Matrix4d t_gripper_object_):
    t_gripper_object(t_gripper_object_)
{
}

//////////////////////////////////////////////////
void Grasp::loadFromYml(
	const std::string & file_name,
    const std::string & robot,
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
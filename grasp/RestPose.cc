/*!
    \file src/RestPose.cc
    \brief RestPose tools
    \author João Borrego : jsbruglie
*/

#include "RestPose.hh"

//////////////////////////////////////////////////
void RestPose::loadFromYml(
	const std::string & file_name,
    const std::string & object_name,
    std::vector<ignition::math::Pose3d> & rest_poses)
{   
    try
    {
        double px,py,pz,rr,rp,ry;

        YAML::Node config = YAML::LoadFile(file_name);
        int num_poses = config[object_name].size();
        for (int i = 0; i < num_poses; i++)
        {
            YAML::Node node = config[object_name][i];
            px = node[0].as<double>();
            py = node[1].as<double>();
            pz = node[2].as<double>();
            rr = node[3].as<double>();
            rp = node[4].as<double>();
            ry = node[5].as<double>();
            rest_poses.emplace_back(px,py,pz,rr,rp,ry);
        }
    }
    catch (YAML::Exception& yamlException)
    {
        errorPrintTrace("Unable to parse " << file_name);
    }
}

/////////////////////////////////////////////////
void RestPose::writeToYml(
    const std::string & file_name,
    const std::string & object_name,
    const std::vector<ignition::math::Pose3d> & rest_poses)
{
    YAML::Node root;
    unsigned int size = rest_poses.size();

    // Build YAML node
    for (unsigned int i = 0; i < size; i++)
    {
        ignition::math::Pose3d const *pose = & rest_poses.at(i);
        root[object_name][std::to_string(i)][0] = pose->Pos().X();
        root[object_name][std::to_string(i)][1] = pose->Pos().Y();
        root[object_name][std::to_string(i)][2] = pose->Pos().Z();
        root[object_name][std::to_string(i)][3] = pose->Rot().Roll();
        root[object_name][std::to_string(i)][4] = pose->Rot().Pitch();
        root[object_name][std::to_string(i)][5] = pose->Rot().Yaw();
    }

    std::ofstream fout(file_name);
    if (!fout) {
        errorPrintTrace("Could write to " << file_name);
    } else {
        fout << root;
        debugPrintTrace("All rest poses written to " << file_name);
    }
}

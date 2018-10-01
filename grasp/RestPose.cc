/*!
    \file grasp/RestPose.cc
    \brief RestPose tools

    \author João Borrego : jsbruglie
*/

#include "RestPose.hh"

//////////////////////////////////////////////////
void RestPose::loadFromYml(
    const std::string & file_name,
    const std::string & object_name,
    std::vector<ignition::math::Pose3d> & rest_poses,
    const bool as_pose,
    const bool as_matrix)
{
    try
    {
        YAML::Node config = YAML::LoadFile(file_name);

        if (as_pose)
        {
            int num_poses = config[object_name]["pose"].size();
            double px,py,pz,rr,rp,ry;

            for (int i = 0; i < num_poses; i++)
            {
                YAML::Node node = config[object_name]["pose"][i];
                px = node[0].as<double>();
                py = node[1].as<double>();
                pz = node[2].as<double>();
                rr = node[3].as<double>();
                rp = node[4].as<double>();
                ry = node[5].as<double>();
                rest_poses.emplace_back(px,py,pz,rr,rp,ry);
            }
        }
        else if (as_matrix)
        {
            int num_poses = config[object_name]["tf"].size();
            double m00,m01,m02,m03,m10,m11,m12,m13,m20,m21,m22,m23;

            for (int i = 0; i < num_poses; i++)
            {
                YAML::Node node = config[object_name]["tf"][i];
                m00 = node[0].as<double>();
                m01 = node[1].as<double>();
                m02 = node[2].as<double>();
                m03 = node[3].as<double>();
                m10 = node[4].as<double>();
                m11 = node[5].as<double>();
                m12 = node[6].as<double>();
                m13 = node[7].as<double>();
                m20 = node[8].as<double>();
                m21 = node[9].as<double>();
                m22 = node[10].as<double>();
                m23 = node[11].as<double>();
                ignition::math::Matrix4d mat(
                    m00,m01,m02,m03,
                    m10, m11, m12, m13,
                    m20, m21, m22, m23,
                    0.0, 0.0, 0.0, 1.0);
                rest_poses.emplace_back(mat.Pose());
            }
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
    const std::vector<ignition::math::Pose3d> & rest_poses,
    const bool as_pose,
    const bool as_matrix)
{
    YAML::Node root;
    unsigned int size = rest_poses.size();

    if (as_pose)
    {
        // Build YAML node
        for (int i = 0; i < size; i++)
        {
            ignition::math::Pose3d const *pose = & rest_poses.at(i);
            std::string str_i = std::to_string(i);
            root[object_name]["pose"][str_i][0] = pose->Pos().X();
            root[object_name]["pose"][str_i][1] = pose->Pos().Y();
            root[object_name]["pose"][str_i][2] = pose->Pos().Z();
            root[object_name]["pose"][str_i][3] = pose->Rot().Roll();
            root[object_name]["pose"][str_i][4] = pose->Rot().Pitch();
            root[object_name]["pose"][str_i][5] = pose->Rot().Yaw();
        }
    }
    if (as_matrix)
    {
        for (int i = 0; i < size; i++)
        {
            ignition::math::Matrix4d mat(rest_poses.at(i));
            std::string str_i = std::to_string(i);
            root[object_name]["tf"][str_i][0]  = mat(0,0);
            root[object_name]["tf"][str_i][1]  = mat(0,1);
            root[object_name]["tf"][str_i][2]  = mat(0,2);
            root[object_name]["tf"][str_i][3]  = mat(0,3);
            root[object_name]["tf"][str_i][4]  = mat(1,0);
            root[object_name]["tf"][str_i][5]  = mat(1,1);
            root[object_name]["tf"][str_i][6]  = mat(1,2);
            root[object_name]["tf"][str_i][7]  = mat(1,3);
            root[object_name]["tf"][str_i][8]  = mat(2,0);
            root[object_name]["tf"][str_i][9]  = mat(2,1);
            root[object_name]["tf"][str_i][10] = mat(2,2);
            root[object_name]["tf"][str_i][11] = mat(2,3);
        }
    }

    std::ofstream fout(file_name);
    if (!fout) {
        errorPrintTrace("Could not write to " << file_name);
    } else {
        fout << root;
        debugPrintTrace("All rest poses written to " << file_name);
    }
}

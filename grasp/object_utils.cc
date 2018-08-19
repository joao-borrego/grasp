/*!
    \file grasp/object_utils.cc
    \brief Object related utilities

    \author João Borrego : jsbruglie
*/

#include "object_utils.hh"

/////////////////////////////////////////////////
void spawnModelFromFile(
    gazebo::transport::PublisherPtr pub,
    const std::string & file)
{
    std::ifstream infile {file};
    std::string model_sdf {
        std::istreambuf_iterator<char>(infile),
        std::istreambuf_iterator<char>()
    };
    gazebo::msgs::Factory msg;
    msg.set_sdf(model_sdf);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void spawnModelFromFilename(
    gazebo::transport::PublisherPtr pub,
    ignition::math::Pose3d & pose,
    const std::string & filename)
{
    gazebo::msgs::Factory msg;
    msg.set_sdf_filename(filename);
    gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
    gazebo::msgs::Set(pose_msg, pose);
    msg.set_allocated_pose(pose_msg);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void removeModel(
    gazebo::transport::PublisherPtr pub,
    const std::string & name)
{
    gazebo::msgs::Request *msg;
    msg = gazebo::msgs::CreateRequest("entity_delete", name);
    pub->Publish(*msg);
}

/////////////////////////////////////////////////
void obtainRestPoses(
    const std::string & file_name,
    std::vector<std::string> & targets,
    std::vector<ignition::math::Pose3d> & poses)
{
    ignition::math::Pose3d pose;

    try
    {
        YAML::Node config = YAML::LoadFile(file_name);
        for (const auto & target : targets)
        {
            YAML::Node node = config[target]["rest"];
            double x, y, z, roll, pitch, yaw;
            x     = node[0].as<double>();
            y     = node[1].as<double>();
            z     = node[2].as<double>();
            roll  = node[3].as<double>();
            pitch = node[4].as<double>();
            yaw   = node[5].as<double>();
            poses.emplace_back(x,y,z,roll,pitch,yaw);
        }
    }
    catch (YAML::Exception& yamlException)
    {
        errorPrintTrace("Unable to parse " << file_name);
    }
}


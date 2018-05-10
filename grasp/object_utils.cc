/*!
    \file grasp/object_utils.cc
    \brief Generate candidate grasps

    TODO

    \author João Borrego : jsbruglie
*/

#include "object_utils.hh"

void spawnModelFromFile(
    gazebo::transport::PublisherPtr pub,
    std::string & file)
{
    std::ifstream infile {file};
    std::string model_sdf {
        std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>()
    };
    gazebo::msgs::Factory msg;
    msg.set_sdf(model_sdf);
    pub->Publish(msg);
}

void spawnModelFromFilename(
    gazebo::transport::PublisherPtr pub,
    ignition::math::Pose3d & pose,
    std::string & filename)
{
    gazebo::msgs::Factory msg;
    msg.set_sdf_filename(filename);
    gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
    gazebo::msgs::Set(pose_msg, pose);
    msg.set_allocated_pose(pose_msg);
    pub->Publish(msg);
}

void removeModel(
    gazebo::transport::PublisherPtr pub,
    std::string & name)
{
    gazebo::msgs::Request *msg;
    msg = gazebo::msgs::CreateRequest("entity_delete", name);
    pub->Publish(*msg);
}

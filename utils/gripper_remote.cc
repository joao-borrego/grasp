/*!
    \file utils/gripper_remote.cc
    \brief TODO

    TODO

    \author João Borrego : jsbruglie
*/

#include "gripper_remote.hh"

int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Create the communication node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the object spawner topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<grasp::msgs::Gripper>(GRIPPER_PLUGIN_TOPIC);

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    // Main loop
    std::string line = "";

    while (std::cout << PROMPT)
    {
        getline(std::cin, line);
        std::stringstream input_stream(line);

        // Create a custom message
        grasp::msgs::Gripper msg;

        // Fill the contents of the message
        ignition::math::Pose3d pose(0.1,0.1,0.1,0.1,0.2,0.3);
        gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
        gazebo::msgs::Set(pose_msg, pose);
        msg.set_allocated_pose(pose_msg);

        // Send the message
        pub->Publish(msg);
    }

    // Shut down
    gazebo::client::shutdown();
    return 0;
}

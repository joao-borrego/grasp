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
        // Process command
        getline(std::cin, line);
        std::stringstream input_stream(line);
        std::string command = input_stream.str();
        // Change pose request
        if (command == "pose")
        {
            ignition::math::Pose3d pose(0,0,0.1,0,1.57,0);
            setPose(pub, pose);
        }
        // Change velocity request
        else if (command == "velocity")
        {
            std::vector<double> velocity {0,0,0.8,0,0,0};
            setVelocity(pub, velocity);
        }
        // Open gripper request
        else if (command == "open")
        {
            openGripper(pub);
        }
        // Close gripper request
        else if (command == "close")
        {
            closeGripper(pub);
        }
        // Perform grasp attempt request
        else if (command == "grasp")
        {
           tryGrasp(pub);
        }
    }

    // Shut down
    gazebo::client::shutdown();
    return 0;
}

/////////////////////////////////////////////////
void setPose(gazebo::transport::PublisherPtr pub,
    ignition::math::Pose3d pose)
{
    grasp::msgs::Gripper msg;
    gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
    gazebo::msgs::Set(pose_msg, pose);
    msg.set_allocated_pose(pose_msg);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void setVelocity(gazebo::transport::PublisherPtr pub,
    std::vector<double> & velocity)
{
    grasp::msgs::Gripper msg;
    google::protobuf::RepeatedField<double> data(velocity.begin(), velocity.end());
    msg.mutable_velocity()->Swap(&data);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void openGripper(gazebo::transport::PublisherPtr pub)
{
    grasp::msgs::Gripper msg;
    msg.set_open(true);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void closeGripper(gazebo::transport::PublisherPtr pub)
{
    grasp::msgs::Gripper msg;
    msg.set_open(false);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void tryGrasp(gazebo::transport::PublisherPtr pub)
{
    ignition::math::Pose3d pose(0,0,0.1,0,1.57,0);
    std::vector<double> velocity_lift {0,0,0.8,0,0,0};
    std::vector<double> velocity_stop {0,0,0,0,0,0};
    
    openGripper(pub);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    setPose(pub, pose);
    closeGripper(pub);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    setVelocity(pub, velocity_lift);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    setVelocity(pub, velocity_stop);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    openGripper(pub);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    setPose(pub, pose);
}

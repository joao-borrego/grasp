/*!
    \file utils/hand_remote.cc
    \brief TODO

    TODO

    \author João Borrego : jsbruglie
*/

#include "hand_remote.hh"

int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Create the communication node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the object spawner topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<grasp::msgs::Hand>(HAND_PLUGIN_TOPIC);

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
            ignition::math::Pose3d pose(0.07,0.12,0.1,0,1.57,0);
            setPose(pub, pose);
        }
        // Change velocity request
        else if (command == "up")
        {
            std::vector<double> velocity {0,0,10,0,0,0};
            setVelocity(pub, velocity);
        }
        // Close hand
        else if (command == "close")
        {
            std::vector<double> velocities_close {6,6,6};
            std::vector<double> velocity {0,0,0,0,0,0};
            setJointVelocities(pub, velocities_close);
            setVelocity(pub, velocity);
        }
        // Open hand
        else if (command == "open")
        {
            std::vector<double> velocities_open {0,0,0};
            std::vector<double> velocity {0,0,0,0,0,0};
            setJointVelocities(pub, velocities_open);
            setVelocity(pub, velocity);
        }
        // Perform grasp attempt request
        else if (command == "grasp")
        {
           tryGrasp(pub);
        }
        // Reset everything
        else if (command == "reset")
        {
            reset(pub);
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
    grasp::msgs::Hand msg;
    gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
    gazebo::msgs::Set(pose_msg, pose);
    msg.set_allocated_pose(pose_msg);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void setVelocity(gazebo::transport::PublisherPtr pub,
    std::vector<double> & velocity)
{
    grasp::msgs::Hand msg;
    google::protobuf::RepeatedField<double> data(velocity.begin(), velocity.end());
    msg.mutable_velocity()->Swap(&data);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void setJointVelocities(gazebo::transport::PublisherPtr pub,
    std::vector<double> & velocities)
{
    grasp::msgs::Hand msg;
    google::protobuf::RepeatedField<double> data(velocities.begin(), velocities.end());
    msg.mutable_joint_velocities()->Swap(&data);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void reset(gazebo::transport::PublisherPtr pub)
{
    grasp::msgs::Hand msg;
    msg.set_reset(true);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void tryGrasp(gazebo::transport::PublisherPtr pub)
{
    ignition::math::Pose3d pose(0.07,0.12,0.1,0,1.57,0);
    std::vector<double> velocity_lift {0,0,20,0,0,0};
    std::vector<double> velocity_stop {0,0,0,0,0,0};
    std::vector<double> velocities_close {8,8,8};
    std::vector<double> velocities_open {0,0,0};

    setPose(pub, pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    setJointVelocities(pub, velocities_close);
    setVelocity(pub, velocity_stop);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    setVelocity(pub, velocity_lift);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    setJointVelocities(pub, velocities_open);
    setVelocity(pub, velocity_stop);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    reset(pub);
}
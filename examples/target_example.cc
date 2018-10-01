/*!
    \file examples/target_example.cc
    \brief Target object plugin example

    \author João Borrego : jsbruglie
*/

#include "target_example.hh"

int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Create the communication node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the target plugin request topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<grasp::msgs::TargetRequest>(REQUEST_TOPIC);
    // Subscribe to the grasp target topic and link callback function
    gazebo::transport::SubscriberPtr sub =
        node->Subscribe(RESPONSE_TOPIC, onTargetResponse);

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
            ignition::math::Pose3d pose(0,0,1,0,0,0);
            setPose(pub, pose);
            getRestingPose(pub);
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
    grasp::msgs::TargetRequest msg;
    gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
    gazebo::msgs::Set(pose_msg, pose);
    msg.set_type(SET_POSE);
    msg.set_allocated_pose(pose_msg);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void getRestingPose(gazebo::transport::PublisherPtr pub)
{
    grasp::msgs::TargetRequest msg;
    msg.set_type(GET_REST_POSE);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void onTargetResponse(TargetResponsePtr & _msg)
{
    if (_msg->has_pose())
    {
        ignition::math::Pose3d pose(gazebo::msgs::ConvertIgn(_msg->pose()));
        std::cout << "\nReceived pose " << pose << std::endl;
    }
}

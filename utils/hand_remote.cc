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
            ignition::math::Pose3d pose(0,0.0,0.15,0,0,0);
            setPose(pub, pose);
        }
        // Change velocity request
        else if (command == "lift")
        {
            liftHand(pub);
        }
        // Close hand
        else if (command == "close")
        {
            moveFingers(pub, true);
        }
        // Open hand
        else if (command == "open")
        {
            moveFingers(pub, false);
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
void moveFingers(gazebo::transport::PublisherPtr pub,
    bool close)
{
    grasp::msgs::Hand msg;
    std::vector<std::string> joints;
    std::vector<double> values;
    double value = close? 1.57 : 0.0; 

    #ifdef SHADOWHAND // Shadowhand
    joints = {
        "rh_FFJ4","rh_FFJ3","rh_FFJ2",
        "rh_MFJ4","rh_MFJ3","rh_MFJ2",
        "rh_RFJ4","rh_RFJ3","rh_RFJ2",
        "rh_FFJ4","rh_LFJ3","rh_LFJ2",
        "rh_THJ5","rh_THJ4","rh_THJ3","rh_THJ2",
    };
    values = {
        0.0, value, value,
        0.0, value, value,
        0.0, value, value,
        0.0, value, value,
        0.0, 0.0, value, value,
    };
    #endif
    #ifndef SHADOWHAND // Vizzy
    joints = {
        "r_thumb_phal_1_joint",
        "r_ind_phal_1_joint",
        "r_med_phal_1_joint"
    };
    values = {value, value, value};
    #endif
    
    for (unsigned int i = 0; i < joints.size(); i++)
    {
        grasp::msgs::Target *target = msg.add_pid_targets();
        target->set_type(POSITION);
        target->set_joint(joints.at(i));
        target->set_value(values.at(i));
    }

    joints = {
        "virtual_px_joint","virtual_py_joint", "virtual_pz_joint",
        "virtual_rr_joint","virtual_rp_joint", "virtual_ry_joint"
    };
    values = {0,0,0,0,0,0};

    for (unsigned int i = 0; i < joints.size(); i++)
    {
        grasp::msgs::Target *target = msg.add_pid_targets();
        target->set_type(VELOCITY);
        target->set_joint(joints.at(i));
        target->set_value(values.at(i));
    }

    pub->Publish(msg);
}

/////////////////////////////////////////////////
void liftHand(gazebo::transport::PublisherPtr pub)
{
    grasp::msgs::Hand msg;
    std::vector<std::string> joints;
    std::vector<double> values;

    joints = {
        "virtual_px_joint","virtual_py_joint", "virtual_pz_joint",
        "virtual_rr_joint","virtual_rp_joint", "virtual_ry_joint"
    };
    values = {0,0,10,0,0,0};

    for (unsigned int i = 0; i < joints.size(); i++)
    {
        grasp::msgs::Target *target = msg.add_pid_targets();
        target->set_type(VELOCITY);
        target->set_joint(joints.at(i));
        target->set_value(values.at(i));
    }
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
    ignition::math::Pose3d pose(0,0,0.1,0,1.57,0);

    setPose(pub, pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    moveFingers(pub, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    liftHand(pub);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    moveFingers(pub, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    reset(pub);
}
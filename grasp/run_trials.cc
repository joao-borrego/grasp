/*!
    \file grasp/run_trials.cc
    \brief Performs grasp trials

    TODO

    \author João Borrego : jsbruglie
*/

#include "run_trials.hh"

/// Global timeout flag
bool g_timeout  {false};
/// Mutex for global timeout flag
std::mutex g_timeout_mutex;
/// Global trial finished flag
bool g_finished {false};
/// Global trial outcome
bool g_success  {false};
/// Mutex for global timeout flag
std::mutex g_finished_mutex;

int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Create the communication node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    // Publish to the hand plugin topic
    gazebo::transport::PublisherPtr pub_hand =
        node->Advertise<HandMsg>(HAND_REQ_TOPIC);
    // Subscribe to the grasp target topic and link callback function
    gazebo::transport::SubscriberPtr sub_hand =
        node->Subscribe(HAND_RES_TOPIC, onHandResponse);
    // Publish to the grasp target plugin topic
    gazebo::transport::PublisherPtr pub_target =
        node->Advertise<TargetRequest>(TARGET_REQ_TOPIC);
    // Subscribe to the grasp target topic and link callback function
    gazebo::transport::SubscriberPtr sub_target =
        node->Subscribe(TARGET_RES_TOPIC, onTargetResponse);
    // Publish to gazebo's factory topic
    gazebo::transport::PublisherPtr pub_factory =
        node->Advertise<gazebo::msgs::Factory>(FACTORY_TOPIC);
    // Publish to gazebo's request topic
    gazebo::transport::PublisherPtr pub_request =
        node->Advertise<gazebo::msgs::Request>(REQUEST_TOPIC);

    // Wait for subscribers to connect
    pub_factory->WaitForConnection();
    pub_request->WaitForConnection();
    pub_hand->WaitForConnection();
    
    // Spawn target object
    std::string model_filename("model://005_tomato_soup_can");
    spawnModelFromFilename(pub_factory, model_filename);
    pub_target->WaitForConnection();

    // Obtain candidate grasps
    std::vector<Grasp> grasps;
    obtainGrasps(grasps);
    // Perform trials
    for (auto candidate : grasps)
    {
        tryGrasp(candidate, pub_hand, pub_target);
    }

    // Remove target object
    std::string model_name("005_tomato_soup_can");
    removeModel(pub_request, model_name);

    // Shut down
    gazebo::client::shutdown();
    return 0;
}
 
/////////////////////////////////////////////////
void setPose(gazebo::transport::PublisherPtr pub,
    ignition::math::Pose3d pose,
    double timeout)
{
    HandMsg msg;
    gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
    gazebo::msgs::Set(pose_msg, pose);
    msg.set_allocated_pose(pose_msg);
    if (timeout > 0) {
        msg.set_timeout(timeout);
    }
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void setVelocity(gazebo::transport::PublisherPtr pub,
    std::vector<double> & velocity,
    double timeout)
{
    HandMsg msg;
    google::protobuf::RepeatedField<double> data(velocity.begin(), velocity.end());
    msg.mutable_velocity()->Swap(&data);
    if (timeout > 0) {
        msg.set_timeout(timeout);
    }
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void setJointVelocities(gazebo::transport::PublisherPtr pub,
    std::vector<double> & velocities,
    double timeout)
{
    HandMsg msg;
    google::protobuf::RepeatedField<double> data(velocities.begin(), velocities.end());
    msg.mutable_joint_velocities()->Swap(&data);
    if (timeout > 0) {
        msg.set_timeout(timeout);
    }
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void getTargetPose(gazebo::transport::PublisherPtr pub)
{
    TargetRequest msg;
    msg.set_type(REQ_GET_POSE);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void reset(gazebo::transport::PublisherPtr pub)
{
    HandMsg msg;
    msg.set_reset(true);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void tryGrasp(
    Grasp & grasp,
    gazebo::transport::PublisherPtr pub_hand,
    gazebo::transport::PublisherPtr pub_target)
{
    std::vector<double> velocity_lift {0,0,5,0,0,0};
    std::vector<double> velocity_stop {0,0,0};
    std::vector<double> velocities_close {5,5,10};

    setPose(pub_hand, grasp.pose, 0.1);
    while (waitForTimeout()) {waitMs(10);}
    setJointVelocities(pub_hand, velocities_close, 0.5);
    while (waitForTimeout()) {waitMs(10);}
    setVelocity(pub_hand, velocity_lift, 0.1);
    while (waitForTimeout()) {waitMs(10);}
    setVelocity(pub_hand, velocity_stop, 0.5);
    while (waitForTimeout()) {waitMs(10);}
    getTargetPose(pub_target);
    while (waitForTrialEnd()) {waitMs(10);}
    
    std::cout << "Success: " << g_success << std::endl;

    reset(pub_hand);
    waitMs(50);
}

//////////////////////////////////////////////////
bool waitForTimeout()
{
    std::lock_guard<std::mutex> lock(g_timeout_mutex);
    if (g_timeout) {
        g_timeout = false;
        return false;
    }
    return true;
}

//////////////////////////////////////////////////
bool waitForTrialEnd()
{
    std::lock_guard<std::mutex> lock(g_finished_mutex);
    if (g_finished) {
        g_finished = false;
        return false;
    }
    return true;
}

/////////////////////////////////////////////////
void onHandResponse(HandMsgPtr & _msg)
{
    if (_msg->has_timeout()) {
        std::lock_guard<std::mutex> lock(g_timeout_mutex);
        g_timeout = true;
    }
}

/////////////////////////////////////////////////
void onTargetResponse(TargetResponsePtr & _msg)
{
    if (_msg->has_pose()) {
        bool success = false;
        ignition::math::Pose3d pose(gazebo::msgs::ConvertIgn(_msg->pose()));
        if (pose.Pos().Z() > Z_LIFTED) {
            success = true;
        }
        std::lock_guard<std::mutex> lock(g_finished_mutex);
        g_finished = true;
        g_success = success;
    }
}

/////////////////////////////////////////////////
void inline waitMs(int delay)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

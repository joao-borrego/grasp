/*!
    \file grasp/run_trials.cc
    \brief Performs grasp trials

    TODO

    \author João Borrego : jsbruglie
*/

#include "run_trials.hh"

// Globals are used for callbacks

/// Global timeout flag
bool g_timeout  {false};
/// Mutex for global timeout flag
std::mutex g_timeout_mutex;
/// Global object resting flag
bool g_resting  {false};
/// Mutex for global object resting flag
std::mutex g_resting_mutex;
/// Global object resting pose
ignition::math::Pose3d g_pose;
/// Global setp finished flag
bool g_finished {false};
/// Mutex for global step finished flag
std::mutex g_finished_mutex;
/// Global trial outcome
bool g_success  {false};

int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Setup communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    std::map<std::string, gazebo::transport::PublisherPtr> pubs;
    std::map<std::string, gazebo::transport::SubscriberPtr> subs;
    setupCommunications(node, pubs, subs);

    // Wait for subscribers to connect
    pubs["factory"]->WaitForConnection();
    pubs["requests"]->WaitForConnection();
    pubs["hand"]->WaitForConnection();
    
    // Spawn camera
    std::string camera_name("rgbd_camera");
    std::string camera_filename = "model://" + camera_name;
    ignition::math::Pose3d camera_pose(0,0,0.8,0,1.57,0);
    spawnModelFromFilename(pubs["factory"], camera_pose, camera_filename);
    pubs["camera"]->WaitForConnection();

    // TODO: Foreach object

    // Spawn target object
    std::string model_name ("Seal");
    std::string model_filename = "model://" + model_name;
    ignition::math::Pose3d model_pose(0,0,0,0,0,0);
    spawnModelFromFilename(pubs["factory"], model_pose, model_filename);
    pubs["target"]->WaitForConnection();

    // Obtain candidate grasps
    std::string cfg_file("grasp/config/seal.grasp.yml");
    std::vector<Grasp> grasps;
    obtainGrasps(cfg_file, grasps);

    // Obtain object resting position
    getTargetPose(pubs["target"], true);
    while (waitingTrigger(g_resting_mutex, g_resting)) {waitMs(10);}

    // Perform trials
    for (auto candidate : grasps)
    {
        tryGrasp(candidate, pubs["hand"], pubs["target"]);
    }

    // Capture and render frame
    captureFrame(pubs["camera"]);
    while (waitingTrigger(g_finished_mutex, g_finished)) {waitMs(10);}
    
    // Cleanup
    removeModel(pubs["request"], camera_name);
    removeModel(pubs["request"], model_name);

    // Shut down
    gazebo::client::shutdown();
    return 0;
}

/////////////////////////////////////////////////
void setupCommunications(
    gazebo::transport::NodePtr & node,
    std::map<std::string, gazebo::transport::PublisherPtr> & pubs,
    std::map<std::string, gazebo::transport::SubscriberPtr> & subs)
{
    // Create the communication node
    node->Init();

    pubs["hand"] = node->Advertise<HandMsg>(HAND_REQ_TOPIC);
    subs["hand"] = node->Subscribe(HAND_RES_TOPIC, onHandResponse);
    
    pubs["target"] = node->Advertise<TargetRequest>(TARGET_REQ_TOPIC);
    subs["target"] = node->Subscribe(TARGET_RES_TOPIC, onTargetResponse);
    
    pubs["contacts"] = node->Advertise<ContactRequest>(CONTACT_REQ_TOPIC);
    subs["contacts"] = node->Subscribe(CONTACT_RES_TOPIC, onContactResponse);

    pubs["camera"] = node->Advertise<CameraRequest>(CAMERA_REQ_TOPIC);
    subs["camera"] = node->Subscribe(CAMERA_RES_TOPIC, onCameraResponse);

    pubs["factory"] = node->Advertise<gazebo::msgs::Factory>(FACTORY_TOPIC);
    pubs["request"] = node->Advertise<gazebo::msgs::Request>(REQUEST_TOPIC);
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
void getTargetPose(gazebo::transport::PublisherPtr pub, bool rest)
{
    TargetRequest msg;
    if (rest)   { msg.set_type(REQ_REST_POSE); }
    else        { msg.set_type(REQ_GET_POSE); }
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
    std::vector<double> velocities_close {10,10,10};

    setPose(pub_hand, grasp.pose, 0.1);
    while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}
    // Check if hand is already in collision

    setJointVelocities(pub_hand, velocities_close, 0.5);
    while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}
    setVelocity(pub_hand, velocity_lift, 0.1);
    while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}
    setVelocity(pub_hand, velocity_stop, 0.5);
    while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}
    getTargetPose(pub_target, false);
    // TODO - Replace by collision check
    while (waitingTrigger(g_finished_mutex, g_finished)) {waitMs(10);}

    std::cout << "Success: " << g_success
        << " - Pose: " << grasp.pose << "\n";

    reset(pub_hand);
    waitMs(50);
}

/////////////////////////////////////////////////
void captureFrame(gazebo::transport::PublisherPtr pub)
{
    CameraRequest msg;
    msg.set_type(REQ_CAPTURE);
    pub->Publish(msg);
}

//////////////////////////////////////////////////
bool waitingTrigger(std::mutex & mutex, bool & trigger)
{
    std::lock_guard<std::mutex> lock(mutex);
    if (trigger) {
        trigger = false;
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
        if (_msg->type() == RES_POSE)
        {
            bool success = false;
            ignition::math::Pose3d pose(gazebo::msgs::ConvertIgn(_msg->pose()));
            if (pose.Pos().Z() > Z_LIFTED) {
                success = true;
            }
            std::lock_guard<std::mutex> lock(g_finished_mutex);
            g_finished = true;
            g_success = success;
        }
        else if (_msg->type() == RES_REST_POSE)
        {
            std::lock_guard<std::mutex> lock(g_resting_mutex);
            g_pose = gazebo::msgs::ConvertIgn(_msg->pose());
            g_resting = true;
        }
    }
}

/////////////////////////////////////////////////
void onContactResponse(ContactResponsePtr & _msg)
{

}

/////////////////////////////////////////////////
void onCameraResponse(CameraResponsePtr & _msg)
{
    std::lock_guard<std::mutex> lock(g_finished_mutex);
    g_finished = true;
}

/////////////////////////////////////////////////
void inline waitMs(int delay)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

/*!
    \file grasp/run_trials.cc
    \brief Performs grasp trials

    TODO

    \author João Borrego : jsbruglie
*/

#include "run_trials.hh"

/// Global timeout flag
bool g_timeout {false};
/// Mutex for global timeout flag
std::mutex g_timeout_mutex;

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

    // Wait for hand plugin to connect
    pub_hand->WaitForConnection();
    // Wait for grasp target plugin to connect
    pub_target->WaitForConnection();

    // Obtain candidate grasps
    std::vector<Grasp> grasps;
    obtainGrasps(grasps);
    // Perform trials
    for (auto candidate : grasps)
    {
        tryGrasp(candidate, pub_hand, pub_target);
    }

    // Shut down
    gazebo::client::shutdown();
    return 0;
}
 
/////////////////////////////////////////////////
void setPose(gazebo::transport::PublisherPtr pub,
    ignition::math::Pose3d pose)
{
    HandMsg msg;
    gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
    gazebo::msgs::Set(pose_msg, pose);
    msg.set_allocated_pose(pose_msg);
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
    std::vector<double> & velocities)
{
    HandMsg msg;
    google::protobuf::RepeatedField<double> data(velocities.begin(), velocities.end());
    msg.mutable_joint_velocities()->Swap(&data);
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
    std::vector<double> velocity_lift {0,0,20,0,0,0};
    std::vector<double> velocity_stop {0,0,0,0,0,0};
    std::vector<double> velocities_close {8,8,8};
    std::vector<double> velocities_open {0,0,0};

    setPose(pub_hand, grasp.pose);
    waitMs(50);
    setJointVelocities(pub_hand, velocities_close);
    setVelocity(pub_hand, velocity_stop);
    waitMs(1000);
    setVelocity(pub_hand, velocity_lift, 1.0);
    while (waitForTimeout()) {waitMs(10);}
    
    //waitMs(3000);
    //setJointVelocities(pub_hand, velocities_open);
    //setVelocity(pub_hand, velocity_stop);
    //waitMs(1000);
    
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

}

/////////////////////////////////////////////////
void inline waitMs(int delay)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

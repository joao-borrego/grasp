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
/// Global hand resting pose
ignition::math::Pose3d g_safe_pose  {0,0,1,0,0,0};
/// Global object pose
ignition::math::Pose3d g_obj_pose   {0,0,0,0,0,0};
/// Global object resting pose
ignition::math::Pose3d g_rest_pose  {0,0,0,0,0,0};
/// Global setp finished flag
bool g_finished {false};
/// Mutex for global step finished flag
std::mutex g_finished_mutex;
/// Global trial outcome
bool g_success  {false};

int main(int _argc, char **_argv)
{
    // List of grasp targets
    std::vector<std::string> targets;

    // Command-line args
    std::map<std::string, std::string> args;
    std::string obj_cfg_dir, grasp_cfg_dir, out_img_dir, out_trials_dir, robot;
    parseArgs(_argc, _argv,
        obj_cfg_dir, grasp_cfg_dir, out_img_dir, out_trials_dir, robot);

    // Obtain target objects
    obtainTargets(targets, obj_cfg_dir);
    if (targets.empty()) {
        errorPrintTrace("No valid objects retrieved from file");
        exit(EXIT_FAILURE);
    }

    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Setup communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    std::map<std::string, gazebo::transport::PublisherPtr> pubs;
    std::map<std::string, gazebo::transport::SubscriberPtr> subs;
    setupCommunications(node, pubs, subs);
    // Interface for hand plugin
    Interface interface;
    // TODO - Create parameter
    interface.init("grasp/config/robots.yml",robot);

    // TODO - Add camera
    // Spawn camera
    /*
    std::string camera_name("rgbd_camera");
    std::string camera_filename = "model://" + camera_name;
    ignition::math::Pose3d camera_pose(0,0,0.8,0,1.57,0);
    spawnModelFromFilename(pubs["factory"], camera_pose, camera_filename);
    pubs["camera"]->WaitForConnection();
    debugPrintTrace("Camera connected");
    */

    // Initial target object pose
    ignition::math::Pose3d model_pose(0,0,0.1,0,0,0);

    std::string model_filename;

    // For each target object
    for (auto const & model_name : targets)
    {
        // Obtain candidate grasps
        model_filename = "model://" + model_name;
        std::string cfg_file(grasp_cfg_dir + 
            model_name + "." + robot + ".grasp.yml");
        std::vector<Grasp> grasps;
        obtainGrasps(cfg_file, robot, grasps);
        if (grasps.empty()) {
            errorPrintTrace("No valid grasps retrieved from file");
            continue;
        }

        // Reset hand pose so it won't collide with target object
        interface.setPose(g_safe_pose);
        interface.raiseHand(0.1); // Avoid hand dropping
        while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}

        // Spawn object
        spawnModelFromFilename(pubs["factory"], model_pose, model_filename);
        pubs["target"]->WaitForConnection();
        debugPrintTrace("Target connected");
        debugPrint("\tObtaining rest pose ");

        // Obtain object resting position
        while (waitingTrigger(g_resting_mutex, g_resting)) {
            getTargetPose(pubs["target"], true);
            waitMs(200);
            debugPrint("." << std::flush);
        }
        debugPrint(" Done!\n");
        debugPrintTrace("Obtained target object pose");

        // Perform trials
        for (auto candidate : grasps)
        {
            tryGrasp(candidate, interface, pubs, model_name);
        }

        // Cleanup
        removeModel(pubs["request"], model_name);
    }

    // TODO - Render RGBD camera frames

    // Capture and render frame
    /*
    captureFrame(pubs["camera"]);
    while (waitingTrigger(g_finished_mutex, g_finished)) {waitMs(10);}    
    */

    // Cleanup
    /*
    removeModel(pubs["request"], camera_name);
    */
    interface.setPose(g_safe_pose);

    // Shut down
    gazebo::client::shutdown();
    return 0;
}

//////////////////////////////////////////////////
const std::string getUsage(const char* argv_0)
{
    return \
        "usage:   " + std::string(argv_0) + " [options]\n" +
        "options: -d <object dataset yaml>\n"  +
        "         -g <grasp candidates yaml>\n" +
        "         -i <image output directory>\n" +
        "         -o <dataset output directory>\n" +
        "         -r <robot>\n";
}

//////////////////////////////////////////////////
void parseArgs(
    int argc,
    char** argv,
    std::string & obj_cfg_dir,
    std::string & grasp_cfg_dir,
    std::string & out_img_dir,
    std::string & out_trials_dir,
    std::string & robot)
{
    int opt;
    bool d, g, i, o, r;

    while ((opt = getopt(argc,argv,"d: g: i: o: r:")) != EOF)
    {
        switch (opt)
        {
            case 'd':
                d = true; obj_cfg_dir = optarg;    break;
            case 'g':
                g = true; grasp_cfg_dir = optarg;  break;
            case 'i':
                i = true; out_img_dir = optarg;    break;
            case 'o':
                o = true; out_trials_dir = optarg; break;
            case 'r':
                r = true; robot = optarg; break;
            case '?':
                std::cerr << getUsage(argv[0]);
            default:
                std::cout << std::endl;
                exit(EXIT_FAILURE);
        }
    }

    if (!d || !g || !i || !o || !r) {
        std::cerr << getUsage(argv[0]);
        exit(EXIT_FAILURE);
    }

    debugPrint("Parameters:\n" <<
        "   Object dataset yaml      '" << obj_cfg_dir << "'\n" <<
        "   Grasp candidates yaml    '" << grasp_cfg_dir << "'\n" <<
        "   Image output directory   '" << out_img_dir << "'\n" <<
        "   Dataset output directory '" << out_trials_dir << "'\n" <<
        "   Robot                    '" << robot << "'\n");
}

/////////////////////////////////////////////////
void setupCommunications(
    gazebo::transport::NodePtr & node,
    std::map<std::string, gazebo::transport::PublisherPtr> & pubs,
    std::map<std::string, gazebo::transport::SubscriberPtr> & subs)
{
    // Create the communication node
    node->Init();

    // Instance publishers and subscribers
    subs["hand"] = node->Subscribe(HAND_RES_TOPIC, onHandResponse);
    pubs["target"] = node->Advertise<TargetRequest>(TARGET_REQ_TOPIC);
    subs["target"] = node->Subscribe(TARGET_RES_TOPIC, onTargetResponse);
    pubs["contact"] = node->Advertise<ContactRequest>(CONTACT_REQ_TOPIC);
    subs["contact"] = node->Subscribe(CONTACT_RES_TOPIC, onContactResponse);
    pubs["camera"] = node->Advertise<CameraRequest>(CAMERA_REQ_TOPIC);
    subs["camera"] = node->Subscribe(CAMERA_RES_TOPIC, onCameraResponse);
    pubs["factory"] = node->Advertise<gazebo::msgs::Factory>(FACTORY_TOPIC);
    pubs["request"] = node->Advertise<gazebo::msgs::Request>(REQUEST_TOPIC);

    // Wait for subscribers to connect
    pubs["factory"]->WaitForConnection();
    pubs["request"]->WaitForConnection();
    pubs["contact"]->WaitForConnection();
    debugPrintTrace("Gazebo connected");
}

/////////////////////////////////////////////////
void obtainTargets(std::vector<std::string> & targets,
    const std::string & file_name)
{
    std::string mesh, name, path;

    try
    {
        YAML::Node config = YAML::LoadFile(file_name);
        for (const auto & kv_pair : config)
        {
            // TODO? - Use proper object for target object data
            name = kv_pair.second["name"].as<std::string>();
            //mesh = kv_pair.second["mesh"].as<std::string>();
            //path = kv_pair.second["path"].as<std::string>();
            targets.push_back(name);
        }
    }
    catch (YAML::Exception& yamlException)
    {
        std::cerr << "Unable to parse " << file_name << "\n";
    }
    debugPrintTrace("Found " << targets.size() << " objects in " << file_name);
}

/////////////////////////////////////////////////
void getContacts(gazebo::transport::PublisherPtr pub,
    const std::string & target,
    const std::string & hand)
{
    ContactRequest msg;
    CollisionRequest *msg_col_gnd = msg.add_collision();
    msg_col_gnd->set_collision1("ground_plane");
    msg_col_gnd->set_collision2(hand);
    CollisionRequest *msg_col_tgt = msg.add_collision();
    msg_col_tgt->set_collision1(target);
    msg_col_tgt->set_collision2(hand);
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
    Interface & interface,
    std::map<std::string, gazebo::transport::PublisherPtr> & pubs,
    const std::string & target)
{
    // Get pose in world frame, given the object pose
    ignition::math::Pose3d hand_pose = grasp.getPose(g_rest_pose);

    debugPrintTrace("\tCandidate " << hand_pose);
    debugPrintTrace("\tTarget " << g_obj_pose);
    if (hand_pose.Pos().Z() < 0) {
        debugPrintTrace("\tPose beneath ground plane. Aborting grasp");
        return;
    }

    // Teleport hand to grasp candidate pose
    // Add the resting position transformation first
    interface.setPose(g_safe_pose, 0.00001);
    while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}
    debugPrintTrace("\tHand moved to safe pose");
    interface.openFingers(0.001);
    while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}
    debugPrintTrace("\tHand opened fingers");
    interface.setPose(hand_pose, 0.00001);
    while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}
    debugPrintTrace("\tHand moved to grasp candidate pose");

    // Check if hand is already in collision
    getContacts(pubs["contact"], target, interface.getRobotName());
    while (waitingTrigger(g_finished_mutex, g_finished)) {waitMs(10);}
    if (!g_success) {
        grasp.success = false;
        errorPrintTrace("\tCollisions detected. Aborting grasp");
        return;
    }
    debugPrintTrace("\tNo collisions detected");

    // Close fingers
    interface.closeFingers(0.5);
    while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}
    debugPrintTrace("\tFingers closed");

    // Lift object
    interface.raiseHand(2.0);
    while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}
    debugPrintTrace("\tObject lifted");

    // Check if object was grasped
    getContacts(pubs["contact"], target, "ground_plane");
    while (waitingTrigger(g_finished_mutex, g_finished)) {waitMs(10);}

    if (g_success) debugPrintTrace("Success - Object lifted");
    else           errorPrintTrace("Failed - Object fell");

    grasp.success = g_success;
    // Resetting interface places target in rest pose
    interface.reset();

    // TODO - Proper synchronisation with reset signal
    waitMs(100);
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
    debugPrint("\tTarget plugin response\n");
    if (_msg->has_pose()) {
        if (_msg->type() == RES_REST_POSE)
        {
            std::lock_guard<std::mutex> lock(g_resting_mutex);
            g_rest_pose = gazebo::msgs::ConvertIgn(_msg->pose());
            g_resting = true;
        }
    }
}

/////////////////////////////////////////////////
void onContactResponse(ContactResponsePtr & _msg)
{
    debugPrint("\tContact plugin response\n");
    std::lock_guard<std::mutex> lock(g_finished_mutex);
    g_finished = true;
    g_success = (_msg->contacts_size() == 0);
}

/////////////////////////////////////////////////
void onCameraResponse(CameraResponsePtr & _msg)
{
    debugPrint("\tCamera plugin response\n");
    std::lock_guard<std::mutex> lock(g_finished_mutex);
    g_finished = true;
}

/////////////////////////////////////////////////
void inline waitMs(int delay)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

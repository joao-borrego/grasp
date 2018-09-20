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
/// Global setp finished flag
bool g_finished {false};
/// Mutex for global step finished flag
std::mutex g_finished_mutex;

/// Global hand resting pose
ignition::math::Pose3d g_safe_pose  {0,0,0.9,0,0,0};
/// Global object pose
ignition::math::Pose3d g_obj_pose   {0,0,0.1,0,0,0};
/// Global object resting pose
ignition::math::Pose3d g_rest_pose  {0,0,0,0,0,0};

/// Global trial outcome
bool g_success  {false};

int main(int _argc, char **_argv)
{

    // List of grasp targets
    std::vector<std::string> targets;

    // YAML node for configs
    Config config;
    parseArgs(_argc, _argv, config);
    debugPrintTrace("Loaded configuration file.");

    // Obtain target objects
    obtainTargets(targets, config["obj_cfg"]);
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
    // Domain randomiser
    Randomiser rand_api(config["randomiser_cfg"]);
    debugPrintTrace("Initialised randomiser.");


    // DEBUG randomiser
    rand_api.randomise();
    return 0;


    // Interface for hand plugin
    Interface interface;
    interface.init(config["robot_cfg"],  config["robot"]);
    debugPrintTrace("Initialised hand interface.");

    std::string model_filename;

    // For each target object
    for (auto const & model_name : targets)
    {
        // Obtain candidate grasps
        model_filename = "model://" + model_name;
        std::string grasp_file(config["grasp_cfg_dir"] +
            model_name + "." + config["robot"] + ".grasp.yml");
        std::vector<Grasp> grasps;
        obtainGrasps(grasp_file, config["robot"], grasps);
        if (grasps.empty()) {
            errorPrintTrace("No valid grasps retrieved from file");
            continue;
        }

        // Reset hand pose so it won't collide with target object
        interface.setPose(g_safe_pose);
        interface.raiseHand(0.1); // Avoid hand dropping
        while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}

        // Spawn object
        spawnModelFromFilename(pubs["factory"], g_obj_pose, model_filename);
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

        // Perform trials
        for (auto candidate : grasps)
        {
            // TODO - Use randomiser
            tryGrasp(candidate, interface, pubs, model_name);
            // Place target in rest pose
            resetTarget(pubs["target"]);
            while (waitingTrigger(g_finished_mutex, g_finished)) {waitMs(10);}
        }

        // Cleanup
        removeModel(pubs["request"], model_name);
    }

    // Cleanup
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
        "options: -c <config yml>";
}

//////////////////////////////////////////////////
void parseArgs(int argc, char** argv, Config & config)
{
    int opt;
    bool c;
    std::string config_path;

    while ((opt = getopt(argc,argv,"c:")) != EOF)
    {
        switch (opt)
        {
            case 'c':
                c = true; config_path = optarg; break;
            case '?':
                std::cerr << getUsage(argv[0]);
            default:
                std::cout << std::endl;
                exit(EXIT_FAILURE);
        }
    }

    if (!c) {
        std::cerr << getUsage(argv[0]);
        exit(EXIT_FAILURE);
    }

    try
    {
        YAML::Node node = YAML::LoadFile(config_path);
        config["obj_cfg"]        = node["obj_cfg"].as<std::string>();
        config["grasp_cfg_dir"]  = node["grasp_cfg_dir"].as<std::string>();
        config["out_trials_dir"] = node["out_trials_dir"].as<std::string>();
        config["robot_cfg"]      = node["robot_cfg"].as<std::string>();
        config["robot"]          = node["robot"].as<std::string>();
        config["randomiser_cfg"] = node["randomiser_cfg"].as<std::string>();
    }
    catch (YAML::Exception& yamlException)
    {
        std::cerr << "Unable to parse " << config_path << "\n";
    }

    debugPrint("Parameters:\n" <<
        "   Configuration yml        '" << config_path << "'\n");
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
void checkHandCollisions(gazebo::transport::PublisherPtr pub,
    const std::string & hand,
    std::vector<std::string> & targets)
{
    ContactRequest msg;
    for (const auto & target : targets)
    {
        CollisionRequest *msg_col_gnd = msg.add_collision();
        msg_col_gnd->set_collision1(target);
        msg_col_gnd->set_collision2(hand);
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
void resetTarget(gazebo::transport::PublisherPtr pub)
{
    TargetRequest msg;
    msg.set_type(REQ_RESET);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void tryGrasp(
    Grasp & grasp,
    Interface & interface,
    std::map<std::string, gazebo::transport::PublisherPtr> & pubs,
    const std::string & target)
{
    std::vector<std::string> target_and_ground {target, "ground_plane"};
    std::vector<std::string> ground {"ground_plane"};

    // Get pose in world frame, given the object pose
    ignition::math::Pose3d hand_pose = grasp.getPose(g_rest_pose);

    debugPrintTrace("\tCandidate " << hand_pose);
    if (hand_pose.Pos().Z() < 0) {
        debugPrintTrace("\tPose beneath ground plane. Aborting grasp");
        return;
    }

    // Teleport hand to grasp candidate pose
    // Add the resting position transformation first
    interface.setPose(g_safe_pose, 0.0001);
    while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}
    debugPrintTrace("\tHand moved to safe pose");
    interface.openFingers(0.001);
    while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}
    debugPrintTrace("\tHand opened fingers");
    interface.setPose(hand_pose, 0.00001);
    while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}
    debugPrintTrace("\tHand moved to grasp candidate pose");

    // Check if hand is already in collision
    checkHandCollisions(pubs["contact"],
        interface.getRobotName(), target_and_ground);
    while (waitingTrigger(g_finished_mutex, g_finished)) {waitMs(10);}
    if (!g_success) {
        grasp.success = false;
        errorPrintTrace("\tCollisions detected. Aborting grasp");
        return;
    }
    debugPrintTrace("\tNo collisions detected");

    // Close fingers
    interface.closeFingers(1.0);
    while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}
    debugPrintTrace("\tFingers closed");

    // Lift object
    interface.raiseHand(5.0);
    while (waitingTrigger(g_timeout_mutex, g_timeout)) {waitMs(10);}
    debugPrintTrace("\tHand lifted");

    // Check if object was grasped
    checkHandCollisions(pubs["contact"], target, ground);
    while (waitingTrigger(g_finished_mutex, g_finished)) {waitMs(10);}

    if (g_success) debugPrintTrace("Success - Object is not on the ground");
    else           errorPrintTrace("Failed - Object is on the ground");

    grasp.success = g_success;
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
    debugPrint("\tTarget plugin response.\n");
    if (_msg->has_pose()) {
        if (_msg->type() == RES_REST_POSE)
        {
            std::lock_guard<std::mutex> lock(g_resting_mutex);
            g_rest_pose = gazebo::msgs::ConvertIgn(_msg->pose());
            g_resting = true;
        }
        else if (_msg->type() == RES_POSE)
        {
            std::lock_guard<std::mutex> lock(g_finished_mutex);
            g_finished = true;
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

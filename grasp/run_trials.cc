/*!
    \file grasp/run_trials.cc
    \brief Performs grasp trials

    Run full dynamic simulated grasp trials in Gazebo

    \author João Borrego : jsbruglie
*/

#include "run_trials.hh"

// Globals are used for callbacks

// Condition variable for exclusive threaded access
std::condition_variable g_timeout_var;
/// Mutex for condition variables
std::mutex g_timeout_mutex;

/// Global hand resting pose
ignition::math::Pose3d g_safe_pose  {0,0,0.9,0,0,0};
/// Global object pose
ignition::math::Pose3d g_obj_pose   {0.5,0.5,0.1,0,0,0};
/// Global object resting pose
ignition::math::Pose3d g_rest_pose  {0,0,0,0,0,0};

/// Global trial outcome
bool g_success  {false};

int main(int _argc, char **_argv)
{
    // List of grasp targets
    std::vector<std::string> targets;

    // YAML node for configs
    Config cfg;
    parseArgs(_argc, _argv, cfg);
    int num_trials = std::atoi(cfg["num_trials"].c_str());
    bool use_randomiser = cfg["use_randomiser"] == "True";

    debugPrintTrace("Loaded configuration file.");

    // Obtain target objects
    obtainTargets(targets, cfg["obj_cfg"]);
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
    Randomiser rand_api(cfg["randomiser_cfg"]);
    debugPrintTrace("Initialised randomiser.");

    // Interface for hand plugin
    Interface interface;
    interface.init(cfg["robot_cfg"],  cfg["robot"]);
    debugPrintTrace("Initialised hand interface.");

    std::string model_filename;

    // For each target object
    for (auto const & model_name : targets)
    {
        std::vector<Grasp> grasps;
        model_filename = "model://" + model_name;

        // Obtain candidate grasps
        if (!importGrasps(cfg["grasp_cfg_dir"], cfg["robot"], model_name, grasps)) {
            continue; // No valid grasp retrieved from file
        }

        // Reset hand pose so it won't collide with target object
        interface.setPose(g_safe_pose);

        interface.raiseHand(0.1); // Avoid hand dropping
        waitForTrigger(500);

        // Obtain rest pose from file
        std::string rest_file(cfg["rest_cfg_dir"] + model_name + ".rest.yml");
        std::vector<ignition::math::Pose3d> rest_poses;
        RestPose::loadFromYml(rest_file, model_name, rest_poses);
        if (rest_poses.empty()) {
            errorPrintTrace("No valid rest poses retrieved from file.");
            continue;
        }
        g_rest_pose = rest_poses.at(0);

        // Spawn object
        debugPrintTrace("Spawning target object");
        spawnModelFromFilename(pubs["factory"], g_rest_pose, model_filename);
        pubs["target"]->WaitForConnection();
        debugPrintTrace("Target connected");
        rand_api.setTargetName(model_name);

        interface.setPose(g_safe_pose, 5.0);
        waitForTrigger(5000);

        // Perform trials
        for (auto & candidate : grasps)
        {
            double metric = 0.0;

            for (int i = 0; i < num_trials; i++)
            {
                debugPrintTrace("Object " << model_name << " Grasp " <<
                    candidate.id + 1<< "/" << grasps.size() <<
                    " - Trial " << i + 1 << "/" << num_trials);

                // Randomise scene
                if (use_randomiser) { rand_api.randomise(); }
                // Try grasp
                double outcome = tryGrasp(candidate, interface, pubs, model_name);

                // Place target in rest pose, and wait for reply
                resetTarget(pubs["target"]);
                waitForTrigger();
                // If grasp is invalid stop repetitions
                if (outcome == INVALID_GRASP){
                    break;
                }
                metric += outcome;
            }
            if (metric > 0) { candidate.metric = metric / num_trials; }
            else            { candidate.metric = INVALID_GRASP; }
        }
        // Export grasps outcome to file
        exportGraspMetrics(cfg["out_trials_dir"], cfg["robot"], model_name, grasps);

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
        // Required parameters
        config["obj_cfg"]        = node["obj_cfg"].as<std::string>();
        config["grasp_cfg_dir"]  = node["grasp_cfg_dir"].as<std::string>();
        config["rest_cfg_dir"]   = node["rest_cfg_dir"].as<std::string>();
        config["out_trials_dir"] = node["out_trials_dir"].as<std::string>();
        config["robot_cfg"]      = node["robot_cfg"].as<std::string>();
        config["robot"]          = node["robot"].as<std::string>();
        config["randomiser_cfg"] = node["randomiser_cfg"].as<std::string>();
        config["use_randomiser"] = node["use_randomiser"].as<std::string>();
        config["num_trials"]     = node["num_trials"].as<std::string>();
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
            name = kv_pair.second["name"].as<std::string>();
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
bool importGrasps(const std::string & grasp_cfg_dir,
    const std::string & robot,
    const std::string & object_name,
    std::vector<Grasp> & grasps)
{
    std::string file_name(grasp_cfg_dir + object_name + ".grasp.yml");
    Grasp::loadFromYml(file_name, robot, object_name, grasps);
    if (grasps.empty())
    {
        errorPrintTrace("No valid grasps retrieved from file " << file_name);
        return false;
    }
    debugPrintTrace("Retrieved " << grasps.size() << " grasps from file" << file_name);
    return true;
}

/////////////////////////////////////////////////
void exportGraspMetrics(const std::string & trials_out_dir,
    const std::string & robot,
    const std::string & object_name,
    const std::vector<Grasp> & grasps)
{
    std::string file_name(trials_out_dir + object_name + ".metrics.yml");
    Grasp::writeToYml(file_name, robot, object_name, grasps);
    debugPrintTrace("Written grasp metrics to " << file_name);
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
double tryGrasp(
    Grasp & grasp,
    Interface & interface,
    std::map<std::string, gazebo::transport::PublisherPtr> & pubs,
    const std::string & target)
{
    // Aux vectors for collision check queries
    std::vector<std::string> target_and_ground {target, "ground_plane"};
    std::vector<std::string> ground {"ground_plane"};

    // Grasp outcome return var
    double metric = 0.0;

    // Get pose in world frame, given the object pose
    ignition::math::Pose3d hand_pose;
    ignition::math::Pose3d t_gripper_object = grasp.t_gripper_object.Pose();
    ignition::math::Pose3d t_base_gripper = interface.getTransformBaseGripper().Pose();
    hand_pose = t_base_gripper + t_gripper_object + g_rest_pose;

    //debugPrintTrace("\tCandidate " << hand_pose);
    if (hand_pose.Pos().Z() < 0) {
        //debugPrintTrace("\tPose beneath ground plane. Aborting grasp");
        return false;
    }

    // Teleport hand to grasp candidate pose
    // Add the resting position transformation first

    interface.setPose(g_safe_pose, 0.5);
    waitForTrigger(1000);
    //debugPrintTrace("\tHand moved to safe pose");

    interface.openFingers(0.0001, true);
    waitForTrigger(1000);
    //debugPrintTrace("\tHand opened fingers");

    interface.setPose(hand_pose, 0.00001);
    waitForTrigger(1000);
    //debugPrintTrace("\tHand moved to grasp candidate pose");

    // DEBUG pose
    //std::cin.ignore();

    // Check if hand is already in collision
    checkHandCollisions(pubs["contact"],
        interface.getRobotName(), target_and_ground);
    waitForTrigger(1000);
    if (!g_success) {
        errorPrintTrace("\tCollisions detected. Aborting grasp");
        return INVALID_GRASP;
    }
    //debugPrintTrace("\tNo collisions detected");

    interface.closeFingers(2.0, true);
    waitForTrigger(10000);
    //debugPrintTrace("\tFingers closed");

    interface.raiseHand(5.0);
    waitForTrigger(30000);
    //debugPrintTrace("\tHand lifted");

    // Check if object was grasped
    checkHandCollisions(pubs["contact"], target, ground);
    waitForTrigger(1000);

    metric = (g_success)? 1.0 : 0.0;
    //if (g_success) debugPrintTrace("Success - Object is not on the ground");
    //else           errorPrintTrace("Failed - Object is on the ground");

    // Reset hand
    interface.reset();

    return metric;
}

//////////////////////////////////////////////////
void waitForTrigger(int timeout)
{

    std::unique_lock<std::mutex> lock(g_timeout_mutex);
    if (timeout <= 0) {
        //debugPrintTrace("Waiting for trigger");
        g_timeout_var.wait(lock);
    } else {
        //debugPrintTrace("Waiting for at most " << timeout << " ms");
        g_timeout_var.wait_for(lock, std::chrono::milliseconds(timeout));
    }
}

/////////////////////////////////////////////////
void onHandResponse(HandMsgPtr & _msg)
{
    //debugPrint("\tHand plugin response.\n");
    if (_msg->has_timeout())
    {
        std::lock_guard<std::mutex> lock(g_timeout_mutex);
        g_timeout_var.notify_one();
    }
}

/////////////////////////////////////////////////
void onTargetResponse(TargetResponsePtr & _msg)
{
    //debugPrint("\tTarget plugin response.\n");
    if (_msg->type() == RES_POSE && _msg->has_pose())
    {
        std::lock_guard<std::mutex> lock(g_timeout_mutex);
        g_timeout_var.notify_one();
    }
}

/////////////////////////////////////////////////
void onContactResponse(ContactResponsePtr & _msg)
{
    //debugPrint("\tContact plugin response\n");
    std::lock_guard<std::mutex> lock(g_timeout_mutex);
    g_success = (_msg->contacts_size() == 0);
    g_timeout_var.notify_one();
}

/////////////////////////////////////////////////
void inline waitMs(int delay)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

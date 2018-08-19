/*!
    \file grasp/gen_rest_poses.cc
    \brief Obtain object rest poses

    Drop each object in dataset from given pose and record rest pose

    \author João Borrego : jsbruglie
*/

#include "gen_rest_poses.hh"

// Globals are used for callbacks

/// Global object resting flag
bool g_resting  {false};
/// Mutex for global object resting flag
std::mutex g_resting_mutex;

/// Global initial object pose
const ignition::math::Pose3d INIT_POSE  {0,0,0.1,0,0,0};
/// Global object resting pose
ignition::math::Pose3d g_rest_pose      {0,0,0,0,0,0};

int main(int _argc, char **_argv)
{
    // List of grasp targets
    std::vector<std::string> targets;

    // Command-line args
    std::map<std::string, std::string> args;
    std::string obj_cfg_dir, out_rest_dir;
    parseArgs(_argc, _argv, obj_cfg_dir, out_rest_dir);

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

    std::string model_filename;
    ignition::math::Pose3d init_pose = INIT_POSE;

    // For each target object
    for (auto const & model_name : targets)
    {
        // Spawn object
        model_filename = "model://" + model_name;
        spawnModelFromFilename(pubs["factory"], init_pose, model_filename);
        pubs["target"]->WaitForConnection();
        debugPrintTrace(model_name << " - Target connected");

        // Obtain object resting position
        debugPrint("\t" << model_name << " - Obtaining rest pose ");
        while (waitingTrigger(g_resting_mutex, g_resting)) {
            getTargetRestPose(pubs["target"]);
            waitMs(200);
            debugPrint("." << std::flush);
        }
        debugPrint(" Done!\n");
        debugPrintTrace("Rest pose: " << g_rest_pose);

        // Cleanup
        removeModel(pubs["request"], model_name);
        // Prevent spawning next object without first removing current
        waitMs(500);
    }

    debugPrintTrace("All rest poses obtained!");

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
        "         -g <rest poses output yaml>\n";
}

//////////////////////////////////////////////////
void parseArgs(
    int argc,
    char** argv,
    std::string & obj_cfg_dir,
    std::string & out_rest_dir)
{
    int opt;
    bool d, o;

    while ((opt = getopt(argc,argv,"d: o:")) != EOF)
    {
        switch (opt)
        {
            case 'd':
                d = true; obj_cfg_dir = optarg;    break;
            case 'o':
                o = true; out_rest_dir = optarg; break;
            case '?':
                std::cerr << getUsage(argv[0]);
            default:
                std::cout << std::endl;
                exit(EXIT_FAILURE);
        }
    }

    if (!d || !o) {
        std::cerr << getUsage(argv[0]);
        exit(EXIT_FAILURE);
    }

    debugPrint("Parameters:\n" <<
        "   Object dataset yaml      '" << obj_cfg_dir << "'\n" <<
        "   Rest pose output yaml    '" << out_rest_dir << "'\n");
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
    pubs["target"] = node->Advertise<TargetRequest>(TARGET_REQ_TOPIC);
    subs["target"] = node->Subscribe(TARGET_RES_TOPIC, onTargetResponse);
    pubs["factory"] = node->Advertise<gazebo::msgs::Factory>(FACTORY_TOPIC);
    pubs["request"] = node->Advertise<gazebo::msgs::Request>(REQUEST_TOPIC);

    // Wait for subscribers to connect
    pubs["factory"]->WaitForConnection();
    pubs["request"]->WaitForConnection();
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
void getTargetRestPose(gazebo::transport::PublisherPtr pub)
{
    TargetRequest msg;
    msg.set_type(REQ_REST_POSE);
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
void onTargetResponse(TargetResponsePtr & _msg)
{
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
void inline waitMs(int delay)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

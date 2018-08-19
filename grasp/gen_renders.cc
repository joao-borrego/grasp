/*!
    \file grasp/gen_renders.cc
    \brief Obtain camera render per grasp candidate

    TODO

    \author João Borrego : jsbruglie
*/

#include "gen_renders.hh"

// Globals are used for callbacks

/// Global setp finished flag
bool g_finished {false};
/// Mutex for global step finished flag
std::mutex g_finished_mutex;

int main(int _argc, char **_argv)
{
    // List of grasp targets
    std::vector<std::string> targets;
    // List of target rest poses
    std::vector<ignition::math::Pose3d> rest_poses;

    // Command-line args
    std::map<std::string, std::string> args;
    std::string obj_cfg_dir, obj_rest_dir, grasp_cfg_dir, out_img_dir, robot;
    parseArgs(_argc, _argv,
        obj_cfg_dir, obj_rest_dir, grasp_cfg_dir, out_img_dir, robot);

    // Obtain target objects
    obtainTargets(targets, obj_cfg_dir);
    if (targets.empty()) {
        errorPrintTrace("No valid objects retrieved from file");
        exit(EXIT_FAILURE);
    }
    // Obtain target object resting poses
    obtainRestPoses(obj_rest_dir, targets, rest_poses);
    if (rest_poses.size() != targets.size()) {
        errorPrintTrace("Invalid rest poses");
        exit(EXIT_FAILURE);
    }

    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Setup communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    std::map<std::string, gazebo::transport::PublisherPtr> pubs;
    std::map<std::string, gazebo::transport::SubscriberPtr> subs;
    setupCommunications(node, pubs, subs);

    // Spawn camera
    std::string camera_name("rgbd_camera");
    std::string camera_filename = "model://" + camera_name;
    ignition::math::Pose3d camera_pose(0,0,0.8,0,1.57,0);
    spawnModelFromFilename(pubs["factory"], camera_pose, camera_filename);
    pubs["camera"]->WaitForConnection();
    debugPrintTrace("Camera connected");

    std::string model_filename;

    // For each target object
    for (unsigned int i; i < targets.size(); i++)
    {
        std::string model_name = targets.at(i);
        ignition::math::Pose3d rest_pose = rest_poses.at(i);

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

        // Spawn object
        spawnModelFromFilename(pubs["factory"], rest_pose, model_filename);
        pubs["target"]->WaitForConnection();
        debugPrintTrace(model_name << " - Target connected");
        waitMs(1000);

        // Render frame per grasp candidate
        for (auto candidate : grasps)
        {
            // Move camera
            // Ensure camera was moved
            // Capture frame
            // Ensure frame was stored
        }

        // Cleanup
        removeModel(pubs["request"], model_name);
        waitMs(200);
    }

    // Cleanup
    removeModel(pubs["request"], camera_name);

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
        "         -p <object rest poses yaml>\n" +
        "         -g <grasp candidates yaml>\n" +
        "         -i <image output directory>\n" +
        "         -r <robot>\n";
}

//////////////////////////////////////////////////
void parseArgs(
    int argc,
    char** argv,
    std::string & obj_cfg_dir,
    std::string & obj_rest_dir,
    std::string & grasp_cfg_dir,
    std::string & out_img_dir,
    std::string & robot)
{
    int opt;
    bool d, p, g, i, r;

    while ((opt = getopt(argc,argv,"d: p: g: i: r:")) != EOF)
    {
        switch (opt)
        {
            case 'd':
                d = true; obj_cfg_dir = optarg;    break;
            case 'p':
                p = true; obj_rest_dir = optarg;   break;
            case 'g':
                g = true; grasp_cfg_dir = optarg;  break;
            case 'i':
                i = true; out_img_dir = optarg;    break;
            case 'r':
                r = true; robot = optarg; break;
            case '?':
                std::cerr << getUsage(argv[0]);
            default:
                std::cout << std::endl;
                exit(EXIT_FAILURE);
        }
    }

    if (!d || !p || !g || !i || !r) {
        std::cerr << getUsage(argv[0]);
        exit(EXIT_FAILURE);
    }

    debugPrint("Parameters:\n" <<
        "   Object dataset yaml      '" << obj_cfg_dir << "'\n" <<
        "   Object rest poses yaml   '" << obj_rest_dir << "'\n" <<
        "   Grasp candidates yaml    '" << grasp_cfg_dir << "'\n" <<
        "   Image output directory   '" << out_img_dir << "'\n" <<
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
    pubs["factory"] = node->Advertise<gazebo::msgs::Factory>(FACTORY_TOPIC);
    pubs["request"] = node->Advertise<gazebo::msgs::Request>(REQUEST_TOPIC);
    pubs["camera"] = node->Advertise<CameraRequest>(CAMERA_REQ_TOPIC);
    subs["camera"] = node->Subscribe(CAMERA_RES_TOPIC, onCameraResponse);
    pubs["target"] = node->Advertise<TargetRequest>(TARGET_REQ_TOPIC);

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

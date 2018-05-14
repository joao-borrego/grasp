/*!
    \file plugins/RGBDCameraPlugin.cc
    \brief RGBD camera Gazebo plugin implementation

    // TODO

    \author João Borrego : jsbruglie
*/

#include "RGBDCameraPlugin.hh"

namespace gazebo {


/// \brief Class for private Target plugin data.
class RGBDCameraPluginPrivate
{
    /// Gazebo transport node
    public: transport::NodePtr node;
    /// Gazebo RGB topic publisher
    public: transport::PublisherPtr pub_rgb;
    /// Gazebo depth topic publisher
    public: transport::PublisherPtr pub_depth;

    /// Mutex for safe data access
    public: std::mutex mutex;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RGBDCameraPlugin)

/////////////////////////////////////////////////
RGBDCameraPlugin::RGBDCameraPlugin() : ModelPlugin(),
    data_ptr(new RGBDCameraPluginPrivate)
{
    gzmsg << "[RGBDCameraPlugin] Started plugin." << std::endl;
}

/////////////////////////////////////////////////
RGBDCameraPlugin::~RGBDCameraPlugin()
{
    this->newRGBFrameConn.reset();
    this->newDepthFrameConn.reset();
    this->rgb_camera.reset();
    this->depth_camera.reset();
    this->data_ptr->node->Fini();
    gzmsg << "[RGBDCameraPlugin] Unloaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void RGBDCameraPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    std::string rgb_name, depth_name;
    physics::Link_V model_links;
    sensors::SensorManager *sensor_manager;

    this->model = _model;
    this->world = _model->GetWorld();

    // Parameters
    if (_sdf->HasElement(PARAM_RGB)) {
        rgb_name = _sdf->Get<std::string>(PARAM_RGB);
    }
    if (_sdf->HasElement(PARAM_DEPTH)) {
        depth_name = _sdf->Get<std::string>(PARAM_DEPTH);
    }
    if (rgb_name.empty()) {
        gzerr << "RGB sensor name not provided." << std::endl;
        return;
    }
    if (depth_name.empty()) {
        gzerr << "Depth sensor name not provided." << std::endl;
        return;
    }

    // Get camera renderers through the sensor manager
    sensor_manager = sensors::SensorManager::Instance();
    if (!sensor_manager->GetSensor(rgb_name)) {
        gzerr << "RGB camera not found." << std::endl;
        return;
    }
    if (!sensor_manager->GetSensor(depth_name)) {
        gzerr << "Depth camera not found." << std::endl;
        return;
    }

    this->rgb_camera = std::dynamic_pointer_cast<sensors::CameraSensor>(
        sensor_manager->GetSensor(rgb_name))->Camera();
    
    this->depth_camera = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(
        sensor_manager->GetSensor(depth_name))->DepthCamera();

    // Setup publishers
    this->data_ptr->node = transport::NodePtr(new transport::Node());
    this->data_ptr->node->Init();
    this->data_ptr->pub_rgb =
      this->data_ptr->node->Advertise<msgs::ImageStamped>(
        RGB_TOPIC, 1, RGB_PUB_FREQ_HZ);
    this->data_ptr->pub_depth =
      this->data_ptr->node->Advertise<msgs::ImageStamped>(
        DEPTH_TOPIC, 1, DEPTH_PUB_FREQ_HZ);

    // Connect render callback functions
    this->newRGBFrameConn = this->rgb_camera->ConnectNewImageFrame(
        std::bind(&RGBDCameraPlugin::onNewRGBFrame, this,
            std::placeholders::_1, std::placeholders::_2,
            std::placeholders::_3, std::placeholders::_4,
            std::placeholders::_5));
    this->newDepthFrameConn = this->depth_camera->ConnectNewDepthFrame(
        std::bind(&RGBDCameraPlugin::onNewDepthFrame, this,
            std::placeholders::_1, std::placeholders::_2,
            std::placeholders::_3, std::placeholders::_4,
            std::placeholders::_5));

    gzmsg << "[RGBDCameraPlugin] Loaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void RGBDCameraPlugin::onNewRGBFrame(const unsigned char *,
    unsigned int, unsigned int, unsigned int, const std::string &)
{

}

/////////////////////////////////////////////////
void RGBDCameraPlugin::onNewDepthFrame(const float *,
    unsigned int, unsigned int, unsigned int, const std::string &)
{

}

}

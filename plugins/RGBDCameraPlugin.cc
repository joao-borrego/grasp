/*!
    \file plugins/RGBDCameraPlugin.cc
    \brief RGBD camera Gazebo plugin implementation

    // TODO

    \author João Borrego : jsbruglie
*/

#include "RGBDCameraPlugin.hh"

using micro = std::chrono::microseconds;

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
    stop_thread = true;
    if (thread.joinable()) thread.join();

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

    // Create concurrent queue for image frames
    rgb_queue = new ConcurrentQueue<unsigned char*>(10);
    // Start aux thread
    thread = std::thread(&RGBDCameraPlugin::saveRender, this);

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
void RGBDCameraPlugin::onNewRGBFrame(
    const unsigned char *_image,
    unsigned int _width,
    unsigned int _height,
    unsigned int _depth,
    const std::string &_format)
{
    Ogre::ImageCodec::ImageData *imgData;
    size_t size;

    auto start = std::chrono::high_resolution_clock::now();

    size = rendering::Camera::ImageByteSize(_width, _height, _format);

    // Copy frame data to save buffer
    unsigned char *buffer = new unsigned char[size];
    std::memcpy(buffer, _image, size);
    this->rgb_queue->enqueue(buffer);

    /*

    Ogre::Codec * pCodec;
    size_t size, pos;
    std::string _filename = std::to_string(counter) + ".png";

    // Wrap buffer in a chunk
    Ogre::MemoryDataStreamPtr stream(
      new Ogre::MemoryDataStream(const_cast<unsigned char*>(_image),
        size, false));

    // Get codec
    Ogre::String filename = _filename;
    pos = filename.find_last_of(".");
    Ogre::String extension;

    while (pos != filename.length() - 1)
    extension += filename[++pos];

    // Get the codec
    pCodec = Ogre::Codec::getCodec(extension);

    // Write out
    Ogre::Codec::CodecDataPtr codecDataPtr(imgData);
    //pCodec->encodeToFile(stream, filename, codecDataPtr);

    */

    auto finish = std::chrono::high_resolution_clock::now();
    /*
    gzdbg << std::chrono::duration_cast<micro>(finish - start).count()
        << " us\n";
    */
}

/////////////////////////////////////////////////
void RGBDCameraPlugin::onNewDepthFrame(
    const float *_image,
    unsigned int _width,
    unsigned int _height,
    unsigned int _depth,
    const std::string &_format)
{

}

//////////////////////////////////////////////////
void RGBDCameraPlugin::saveRender()
{
    unsigned char *image;
    while(!stop_thread)
    {
        auto start = std::chrono::high_resolution_clock::now();

        rgb_queue->dequeue(image);
        delete [] image;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        auto finish = std::chrono::high_resolution_clock::now();
        /*
        gzdbg << std::chrono::duration_cast<micro>(finish - start).count()
            << " us\n";
        */
    }
}

//////////////////////////////////////////////////
int RGBDCameraPlugin::OgrePixelFormat(const std::string &_format)
{
  int result;

  if (_format == "L8" || _format == "L_INT8")
    result = static_cast<int>(Ogre::PF_L8);
  else if (_format == "R8G8B8" || _format == "RGB_INT8")
    result = static_cast<int>(Ogre::PF_BYTE_RGB);
  else if (_format == "B8G8R8" || _format == "BGR_INT8")
    result = static_cast<int>(Ogre::PF_BYTE_BGR);
  else if (_format == "FLOAT32")
    result = static_cast<int>(Ogre::PF_FLOAT32_R);
  else if (_format == "FLOAT16")
    result = static_cast<int>(Ogre::PF_FLOAT16_R);
  else if ((_format == "BAYER_RGGB8") || (_format == "BAYER_BGGR8") ||
    (_format == "BAYER_GBRG8") || (_format == "BAYER_GRBG8"))
  {
    // let ogre generate rgb8 images for all bayer format requests
    // then post process to produce actual bayer images
    result = static_cast<int>(Ogre::PF_BYTE_RGB);
  }
  else
  {
    gzerr << "Error parsing image format (" << _format
          << "), using default Ogre::PF_R8G8B8\n";
    result = static_cast<int>(Ogre::PF_R8G8B8);
  }

  return result;
}

}

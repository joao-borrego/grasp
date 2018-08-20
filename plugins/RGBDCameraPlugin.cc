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
    /// Gazebo topic publisher
    public: transport::PublisherPtr pub;
    /// Gazebo topic subscriber
    public: transport::SubscriberPtr sub;

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
    gzmsg << "[RGBDCameraPlugin] Closing." << std::endl;

    this->updateConn.reset();
    this->newRGBFrameConn.reset();
    this->newDepthFrameConn.reset();
    this->camera.reset();
    this->data_ptr->node->Fini();

    {
        std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
        stop_thread = true;
    }
    if (thread_rgb.joinable()) thread_rgb.join();
    if (thread_depth.joinable()) thread_depth.join();

    clearQueues();

    gzmsg << "[RGBDCameraPlugin] Unloaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void RGBDCameraPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    std::string camera_name, req_topic, res_topic;
    int queue_size = 0;
    physics::Link_V model_links;
    sensors::SensorManager *sensor_manager;

    this->model = _model;
    this->world = _model->GetWorld();

    // Parse camera name parameters from SDF
    if (_sdf->HasElement(PARAM_CAMERA)) {
        camera_name = _sdf->Get<std::string>(PARAM_CAMERA);
    }
    if (camera_name.empty()) {
        gzerr << "RGBD sensor name not provided." << std::endl;
        return;
    }

    // Get camera renderers through the sensor manager
    sensor_manager = sensors::SensorManager::Instance();
    if (!sensor_manager->GetSensor(camera_name)) {
        gzerr << "Provided RGBD camera not found." << std::endl;
        return;
    }
    this->camera = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(
        sensor_manager->GetSensor(camera_name))->DepthCamera();

    // Parse rendering queue parameter from SDF
    if (_sdf->HasElement(PARAM_QUEUE_SIZE)) {
        // TODO: Check if provided queue size is reasonable
        queue_size = _sdf->Get<int>(PARAM_QUEUE_SIZE);
    }
    if (queue_size <= 0) {
        gzerr << "Invalid render queue size provided." << std::endl;
        return;
    }

    // Parse render output directory from SDF
    if (_sdf->HasElement(PARAM_OUTPUT_DIR)) {
        output_dir = _sdf->Get<std::string>(PARAM_OUTPUT_DIR);
    }
    if (output_dir.empty()) {
        output_dir = DEFAULT_OUTPUT_DIR;
    }

    // Parse output format from SDF
    if (_sdf->HasElement(PARAM_EXTENSION)) {
        output_ext = _sdf->Get<std::string>(PARAM_EXTENSION);
    }
    if (output_ext.empty()) {
        output_ext = DEFAULT_EXTENSION;
    }

    // Parse topic names from SDF
    if (_sdf->HasElement(PARAM_REQ_TOPIC)) {
         req_topic = _sdf->Get<std::string>(PARAM_REQ_TOPIC);
    }
    if (req_topic.empty()) {
        req_topic = DEFAULT_REQ_TOPIC;
    }
    if (_sdf->HasElement(PARAM_RES_TOPIC)) {
        res_topic = _sdf->Get<std::string>(PARAM_RES_TOPIC);
    }
    if (res_topic.empty()) {
        res_topic = DEFAULT_RES_TOPIC;
    }

    // Setup communications
    this->data_ptr->node = transport::NodePtr(new transport::Node());
    this->data_ptr->node->Init();
    // Subcribe to the monitored requests topic
    this->data_ptr->sub = this->data_ptr->node->Subscribe(
        req_topic, &RGBDCameraPlugin::onRequest, this);
    // Setup publisher for the response topic
    this->data_ptr->pub = this->data_ptr->node->
        Advertise<grasp::msgs::CameraResponse>(res_topic);

    // Create concurrent queues for image frames
    rgb_queue = new ConcurrentQueue<unsigned char*>(queue_size);
    depth_queue = new ConcurrentQueue<float*>(queue_size);

    // Start aux threads
    thread_rgb = std::thread(&RGBDCameraPlugin::saveRenderRGB, this);
    thread_depth = std::thread(&RGBDCameraPlugin::saveRenderDepth, this);

    // Connect world update callback function
    this->updateConn = event::Events::ConnectPreRender(
        std::bind(&RGBDCameraPlugin::onUpdate, this));
    // Connect render callback functions
    this->newRGBFrameConn = this->camera->ConnectNewImageFrame(
        std::bind(&RGBDCameraPlugin::onNewRGBFrame, this,
            std::placeholders::_1, std::placeholders::_2,
            std::placeholders::_3, std::placeholders::_4,
            std::placeholders::_5));
    this->newDepthFrameConn = this->camera->ConnectNewDepthFrame(
        std::bind(&RGBDCameraPlugin::onNewDepthFrame, this,
            std::placeholders::_1, std::placeholders::_2,
            std::placeholders::_3, std::placeholders::_4,
            std::placeholders::_5));

    gzdbg << std::endl <<
        "   Render queue size: " << queue_size << std::endl <<
        "   Request topic:     " << req_topic << std::endl <<
        "   Response topic:    " << res_topic << std::endl;

    gzmsg << "[RGBDCameraPlugin] Loaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void RGBDCameraPlugin::onRequest(CameraRequestPtr &_msg)
{
    if (_msg->type() == CAPTURE_REQUEST)
    {
        std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
        capture = true;

        this->camera->SetCaptureDataOnce();
        gzdbg << "Capture request received" << std::endl;
    }
    else if (_msg->type() == MOVE_REQUEST)
    {
        if (_msg->has_pose())
        {
            std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
            new_pose = msgs::ConvertIgn(_msg->pose());
            update_pose = true;
        }
    }
    // TODO? - Alternatively allow each frame to have a specified name
    //  This requires the name to be put on the queue along with the binary data
    else if (_msg->type() == PREFIX_REQUEST)
    {
        if (_msg->has_prefix())
        {
            std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
            output_prefix = _msg->prefix();
            grasp::msgs::CameraResponse msg;
            msg.set_type(PREFIX_RESPONSE);
            this->data_ptr->pub->Publish(msg);
        }
    }
}

/////////////////////////////////////////////////
void RGBDCameraPlugin::onUpdate()
{
    std::lock_guard<std::mutex> lock(this->data_ptr->mutex);

    grasp::msgs::CameraResponse msg;

    if (update_pose)
    {
        this->camera->SetWorldPose(new_pose);
        update_pose = false;

        // DEBUG
        this->model->SetWorldPose(new_pose);

        // Notify subscribers
        gzdbg << "Camera moved to " << new_pose << std::endl;
        msg.set_type(MOVE_RESPONSE);
        this->data_ptr->pub->Publish(msg);
    }

    // Synchronise RGB and depth frames
    if (capture && rgb_captured && depth_captured)
    {
        msg.set_type(CAPTURE_RESPONSE);
        this->data_ptr->pub->Publish(msg);
        capture = rgb_captured = depth_captured = false;
    }
}

/////////////////////////////////////////////////
void RGBDCameraPlugin::onNewRGBFrame(
    const unsigned char *_image,
    unsigned int _width,
    unsigned int _height,
    unsigned int _depth,
    const std::string &_format)
{
    bool save = false;
    {
        std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
        if (capture && !rgb_captured) { save = true; }
    }

    if (save)
    {
        // Copy frame data to save buffer
        size_t size = rendering::Camera::ImageByteSize(_width, _height, _format);
        unsigned char *buffer = new unsigned char[size];
        std::memcpy(buffer, _image, size);
        // Blocking call until queue has room
        this->rgb_queue->enqueue(buffer);

        std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
        rgb_captured = true;
    }
}

/////////////////////////////////////////////////
void RGBDCameraPlugin::onNewDepthFrame(
    const float *_image,
    unsigned int _width,
    unsigned int _height,
    unsigned int _depth,
    const std::string &_format)
{
    bool save = false;
    {
        std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
        if (capture && !depth_captured) { save = true; }
    }

    if (save)
    {
        // Copy frame data to save buffer
        size_t size = rendering::Camera::ImageByteSize(_width, _height, _format);
        float *buffer = new float[size];
        std::memcpy(buffer, _image, size);
        this->depth_queue->enqueue(buffer);        

        std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
        depth_captured = true;
    }
}

//////////////////////////////////////////////////
void RGBDCameraPlugin::saveRenderRGB()
{
    unsigned char *image_rgb;
    unsigned int width = camera->ImageWidth();
    unsigned int height = camera->ImageHeight();
    unsigned int depth = 3;
    std::string format = "R8G8B8";
    unsigned int counter = 0;
    std::string extension(".png");
    std::string prefix("");

    while(true)
    {
        {
            std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
            if (stop_thread) return;
        }
        if (rgb_queue->dequeue(image_rgb))
        {
            {
                std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
                prefix = this->output_prefix;
            }
            std::string filename = output_dir + "/" + prefix + "_rgb_" +
                std::to_string(counter++) + "." + output_ext;
            gzdbg << "Saving rgb image frame to " << filename << std::endl;
            rendering::Camera::SaveFrame(image_rgb, width, height,
                depth, format, filename);
            delete [] image_rgb;

        }
    }
}

//////////////////////////////////////////////////
void RGBDCameraPlugin::saveRenderDepth()
{
    float *image_depth;
    unsigned int width = camera->ImageWidth();
    unsigned int height = camera->ImageHeight();
    unsigned int depth = 1;
    std::string format = "FLOAT32";
    unsigned int counter = 0;
    std::string output_ext_raw = "raw";
    std::string prefix("");

    while(true)
    {
        {
            std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
            if (stop_thread) return;
        }

        if (depth_queue->dequeue(image_depth))
        {
            {
                std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
                prefix = this->output_prefix;
            }
            std::string filename = output_dir + "/" + prefix + "_depth_" +
                std::to_string(counter++) + "." + output_ext_raw;
            gzdbg << "Saving raw depth frame to " << filename << std::endl;
            saveDepthFrame(image_depth, width, height,
                depth, format, filename);
            delete [] image_depth;
        }
    }
}

//////////////////////////////////////////////////
void RGBDCameraPlugin::saveDepthFrame(
    const float *_image,
    unsigned int _width,
    unsigned int _height,
    unsigned int _depth,
    const std::string &_format,
    const std::string &_filename)
{
    // TODO - Improve
    size_t bytes;
    if (_format == "FLOAT32")
    {
        float a_float;
        bytes = _width * _height * _depth * sizeof(a_float);
        
        FILE *write_ptr;
        write_ptr = fopen(_filename.c_str(),"wb");
        if(write_ptr)
        {
            fwrite(_image, bytes, 1, write_ptr);
        }
        else
        {
            gzerr << "Could not open " << filename << std::endl;
        }
    }
}

//////////////////////////////////////////////////
void RGBDCameraPlugin::clearQueues()
{
    unsigned char *rgb_data;
    float *depth_data;

    while (rgb_queue->dequeue(rgb_data))
    {
        delete [] rgb_data;
    }
    while (depth_queue->dequeue(depth_data))
    {
        delete [] depth_data;
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

/*!
    \file plugins/HandPlugin.cc
    \brief Hand Gazebo plugin

    TODO

    \author João Borrego : jsbruglie
*/

#include "HandPlugin.hh"

namespace gazebo {


/// \brief Class for private camera utils plugin data.
class HandPluginPrivate
{
    /// Gazebo transport node
    public: transport::NodePtr node;
    /// Gazebo topic subscriber
    public: transport::SubscriberPtr sub;
    /// Gazebo topic publisher
    public: transport::PublisherPtr pub;

    /// Mutex for safe data access
    public: std::mutex mutex;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(HandPlugin)

/////////////////////////////////////////////////
HandPlugin::HandPlugin() : ModelPlugin(),
    data_ptr(new HandPluginPrivate)
{
    gzmsg << "[HandPlugin] Loaded hand plugin." << std::endl;
}

/////////////////////////////////////////////////
HandPlugin::~HandPlugin()
{
    gzmsg << "[HandPlugin] Unloaded hand plugin." << std::endl;
}

/////////////////////////////////////////////////
void HandPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Validate model to which plugin is attached
    if (_model->GetJointCount() != 0)
    {
        this->model = _model;
    }
    else
    {
        gzerr << "[HandPlugin] Invalid model." << std::endl;
        return;
    }

    // Extract parameters from SDF element

    // Enable/disable gravity
    if (_sdf->HasElement(PARAM_GRAVITY))
    {
        bool gravity_enabled = _sdf->Get<bool>(PARAM_GRAVITY);
        this->model->SetGravityMode(gravity_enabled);
    }

    // Connect to world update event
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&HandPlugin::onUpdate, this));

    // Setup transport node
    this->data_ptr->node = transport::NodePtr(new transport::Node());
    this->data_ptr->node->Init();
    // Subcribe to the monitored requests topic
    this->data_ptr->sub = this->data_ptr->node->Subscribe(REQUEST_TOPIC,
        &HandPlugin::onRequest, this);
}

/////////////////////////////////////////////////
void HandPlugin::onUpdate()
{
    if (this->reset) {
        resetWorld();
        this->reset = false;
    }
}

/////////////////////////////////////////////////
void HandPlugin::onRequest(HandMsgPtr &_msg)
{
    // Handle change pose request
    if (_msg->has_pose()) {
        this->new_pose = msgs::ConvertIgn(_msg->pose());
        this->update_pose = true;
    }
    // Handle reset
    if (_msg->has_reset()) {
        this->reset =  _msg->reset();
    }
}

/////////////////////////////////////////////////
void HandPlugin::imobilise()
{
    // TODO
}

/////////////////////////////////////////////////
void HandPlugin::setPose(ignition::math::Pose3d & _pose)
{
    // TODO
}

/////////////////////////////////////////////////
void HandPlugin::setVelocity(std::vector<double> & _velocity)
{
    // TODO
}

/////////////////////////////////////////////////
void HandPlugin::resetWorld()
{
    physics::WorldPtr world = this->model->GetWorld();
    world->SetPhysicsEnabled(false);
    world->Reset();
    world->SetPhysicsEnabled(true);
}

}

/*!
    \file plugins/GripperPlugin.cc
    \brief Gripper Gazebo plugin

    TODO

    \author João Borrego : jsbruglie
*/

#include "GripperPlugin.hh"

namespace gazebo {


/// \brief Class for private camera utils plugin data.
class GripperPluginPrivate
{
    /// Gazebo transport node
    public: transport::NodePtr node;
    /// Camera utils topic subscriber
    public: transport::SubscriberPtr sub;
    /// Camera utils topic publisher
    public: transport::PublisherPtr pub;

    /// Mutex for safe data access
    public: std::mutex mutex;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GripperPlugin)

/////////////////////////////////////////////////
GripperPlugin::GripperPlugin() : ModelPlugin(),
    data_ptr(new GripperPluginPrivate)
{
    gzmsg << "[GripperPlugin] Loaded gripper plugin." << std::endl;
}

/////////////////////////////////////////////////
GripperPlugin::~GripperPlugin()
{
    gzmsg << "[GripperPlugin] Unloaded gripper plugin." << std::endl;
}

/////////////////////////////////////////////////
void GripperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Validate model to which plugin is attached
    if (_model->GetJointCount() != 0)
    {
        this->model = _model;
    }
    else
    {
        gzerr << "[GripperPlugin] Invalid model." << std::endl;
        return;
    }

    // Extract parameters from SDF element

    // Reference links and joints
    if (_sdf->HasElement(PARAM_BASE_LINK) &&
        _sdf->HasElement(PARAM_LEFT_LINK) &&
        _sdf->HasElement(PARAM_RIGHT_LINK) &&
        _sdf->HasElement(PARAM_LEFT_JOINT) &&
        _sdf->HasElement(PARAM_RIGHT_JOINT))
    {
        std::string base_link_name = _sdf->Get<std::string>(PARAM_BASE_LINK);
        std::string left_link_name = _sdf->Get<std::string>(PARAM_LEFT_LINK);
        std::string right_link_name = _sdf->Get<std::string>(PARAM_RIGHT_LINK);
        std::string left_joint_name = _sdf->Get<std::string>(PARAM_LEFT_JOINT);
        std::string right_joint_name = _sdf->Get<std::string>(PARAM_RIGHT_JOINT);

        this->base_link = _model->GetLink(base_link_name);
        this->left_link = _model->GetLink(left_link_name);
        this->right_link = _model->GetLink(right_link_name);
        this->left_joint = _model->GetJoint(left_joint_name);
        this->right_joint = _model->GetJoint(right_joint_name);

        if (!this->base_link || !this->left_link || !this->right_link ||
            !this->left_joint || !this->right_joint)
        {
            gzerr << "[GripperPlugin] Invalid reference link or joint specified." << std::endl;
            return;
        }
    }
    else
    {
        gzerr << "[GripperPlugin] Unspecified reference link or joint." << std::endl;
        return;
    }

    // Fix gripper to world link
    

    
    // Enable/disable gravity
    if (_sdf->HasElement(PARAM_GRAVITY))
    {
        bool gravity_enabled = _sdf->Get<bool>(PARAM_GRAVITY);
        this->model->SetGravityMode(gravity_enabled);
    }

    // Connect to world update event
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GripperPlugin::onUpdate, this));

    // Setup transport node
    this->data_ptr->node = transport::NodePtr(new transport::Node());
    this->data_ptr->node->Init();
    // Subcribe to the monitored requests topic
    this->data_ptr->sub = this->data_ptr->node->Subscribe(REQUEST_TOPIC,
        &GripperPlugin::onRequest, this);
}

/////////////////////////////////////////////////
void GripperPlugin::onUpdate()
{

}

/////////////////////////////////////////////////
void GripperPlugin::onRequest(GripperMsgPtr &_msg)
{

}

/////////////////////////////////////////////////
void GripperPlugin::open()
{

}

/////////////////////////////////////////////////
void GripperPlugin::close()
{

}

}

/*!
    \file plugins/HandPlugin.cc
    \brief Hand Gazebo plugin

    TODO

    \author João Borrego : jsbruglie
*/

#include "HandPlugin.hh"

namespace gazebo {

/// \brief Class for joint and respective mimics data.
/// TODO: Maybe refine data structure?
class FingerJoint
{
    /// Actuated joint 
    public: physics::JointPtr actuated;
    /// Vector of mimic joints
    public: std::vector<physics::JointPtr> mimic;
    /// Vector of corresponding mimic joint's multipliers
    public: std::vector<double> multipliers;

    public: FingerJoint(physics::JointPtr joint)
    {
        this->actuated = joint;
    }
};

/// \brief Class for private hand plugin data.
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
    gzmsg << "[HandPlugin] Started hand plugin." << std::endl;
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

    // Finger joints
    if (loadFingerJoints(_sdf) != true) return;
    // Virtual joints for unconstrained movement
    if (loadVirtualJoints(_sdf) != true) return;
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

    gzmsg << "[HandPlugin] Loaded hand plugin." << std::endl;
}

/////////////////////////////////////////////////
bool HandPlugin::loadMimicJoints(sdf::ElementPtr _sdf, FingerJoint & joint)
{
    std::string mimic_name;
    double multiplier;

    if (_sdf->HasElement(PARAM_MIMIC_JOINT))
    {
        sdf::ElementPtr mimic_sdf;
        for (mimic_sdf = _sdf->GetElement(PARAM_MIMIC_JOINT);
            mimic_sdf != NULL;
            mimic_sdf = mimic_sdf->GetNextElement())
        {
            if (mimic_sdf->HasAttribute(PARAM_NAME) && 
                mimic_sdf->HasAttribute(PARAM_MULTIPLIER))
            {
                mimic_sdf->GetAttribute(PARAM_NAME)->Get<std::string>(mimic_name);
                mimic_sdf->GetAttribute(PARAM_MULTIPLIER)->Get<double>(multiplier);
                joint.mimic.push_back(this->model->GetJoint(mimic_name));
                joint.multipliers.push_back(multiplier);

                gzdbg << "Mimic joint " << mimic_name << " k = " << multiplier << std::endl;
            }
            else if (mimic_sdf->GetName() == PARAM_MIMIC_JOINT)
            {
                gzerr << "[HandPlugin] No joint name provided." << std::endl;
                return false;
            }
        } 
    }
}

/////////////////////////////////////////////////
bool HandPlugin::loadFingerJoints(sdf::ElementPtr _sdf)
{
    std::string joint_name;
    int inserted = 0;

    if (_sdf->HasElement(PARAM_FINGER_JOINT))
    {
        sdf::ElementPtr joint_sdf;
        for (joint_sdf = _sdf->GetElement(PARAM_FINGER_JOINT);
            joint_sdf != NULL;
            joint_sdf = joint_sdf->GetNextElement())
        {
            if (joint_sdf->HasAttribute(PARAM_NAME))
            {
                joint_sdf->GetAttribute(PARAM_NAME)->Get<std::string>(joint_name);
                this->finger_joints.emplace_back(this->model->GetJoint(joint_name));
                gzdbg << "Actuated joint " << joint_name << std::endl;
                loadMimicJoints(joint_sdf, this->finger_joints.at(inserted++));
            }
            else if (joint_sdf->GetName() == PARAM_FINGER_JOINT)
            {
                gzerr << "[HandPlugin] No joint name provided." << std::endl;
                return false;
            }
        } 
    }
    else
    {
        gzerr << "[HandPlugin] No joints specified." << std::endl;
        return false;
    }
    return true;
}

/////////////////////////////////////////////////
bool HandPlugin::loadVirtualJoints(sdf::ElementPtr _sdf)
{
    if (_sdf->HasElement(PARAM_VIRTUAL_JOINTS))
    {
        std::string virtual_joints_str = _sdf->Get<std::string>(PARAM_VIRTUAL_JOINTS);

        // Convert string to array of strings
        std::vector<std::string> virtual_joint_names;
        boost::split(virtual_joint_names, virtual_joints_str,
            boost::is_any_of(" "), boost::token_compress_on);
        int num_virtual_joints = virtual_joint_names.size();

        for (auto joint_name : virtual_joint_names) {
            this->virtual_joints.push_back(this->model->GetJoint(joint_name));
        }

        if (this->virtual_joints.size() != num_virtual_joints)
        {
            gzerr << "[HandPlugin] Invalid reference link or joint specified." << std::endl;
            return false;
        }
    }
    else
    {
        gzerr << "[HandPlugin] Unspecified reference link or joint." << std::endl;
        return false;
    }
    return true;
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

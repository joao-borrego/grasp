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
    gzmsg << "[HandPlugin] Started plugin." << std::endl;
}

/////////////////////////////////////////////////
HandPlugin::~HandPlugin()
{
    gzmsg << "[HandPlugin] Unloaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void HandPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Validate model to which plugin is attached
    if (_model->GetJointCount() != 0)
    {
        this->model = _model;
        this->world = _model->GetWorld();
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

    gzmsg << "[HandPlugin] Loaded plugin." << std::endl;
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
    std::lock_guard<std::mutex> lock(this->data_ptr->mutex);

    uint32_t now = this->world->Iterations();
    if (now >= this->last_updated + this->update_rate)
    {
        this->last_updated = now;
        if (this->update_joint_velocities) {
            setJointVelocities(this->new_joint_velocities);
            //this->update_joint_velocities = false;
        }
    }

    if (this->update_pose) {
        setPose(this->new_pose);
        this->update_pose = false;
    }
    if (this->update_velocity) {
        setVelocity(this->new_velocity);
        this->update_velocity = false;
    }
    if (this->reset) {
        this->imobilise();
        resetWorld();
        this->last_updated = 0;
        this->reset = false;
    }

}

/////////////////////////////////////////////////
void HandPlugin::onRequest(HandMsgPtr &_msg)
{
    // Handle change pose request
    if (_msg->has_pose()) {
        this->new_pose = msgs::ConvertIgn(_msg->pose());
        std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
        this->update_pose = true;
    }
    // Handle change velocity request
    if (_msg->velocity_size() > 0) {
        std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
        this->new_velocity.clear();
        for (const auto velocity : _msg->velocity()) {
            this->new_velocity.push_back(velocity);
        }
        this->update_velocity = true;
    }
    // Handle change finger joint velocities request
    if (_msg->joint_velocities_size() > 0) {
        std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
        this->new_joint_velocities.clear();
        if (_msg->joint_velocities_size() == this->finger_joints.size())
        {
            for (const auto velocity : _msg->joint_velocities()) {
                this->new_joint_velocities.push_back(velocity);
            }
            this->update_joint_velocities = true;
        }
        else
        {
            gzdbg << "Invalid joint velocities message" << std::endl;
        }
    }
    // Handle reset
    if (_msg->has_reset()) {
        std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
        this->reset =  _msg->reset();
    }
}

/////////////////////////////////////////////////
void HandPlugin::imobilise()
{
    physics::Link_V links = this->model->GetLinks();
    for (auto link : links) { link->ResetPhysicsStates(); }
    physics::Joint_V joints = this->model->GetJoints();
    for (auto joint : joints) { joint->SetVelocity(0, 0); }

    this->update_velocity = false;
    this->update_joint_velocities = false;
}

/////////////////////////////////////////////////
void HandPlugin::resetJoints()
{
    for (int i = 0; i < this->finger_joints.size(); i++)
    {
        this->finger_joints.at(i).actuated->SetPosition(0, 0);
        for (int j = 0; j < this->finger_joints.at(i).mimic.size(); j++)
        {
            this->finger_joints.at(i).mimic.at(j)->SetPosition(0, 0);
        }
    }
}


/////////////////////////////////////////////////
void HandPlugin::setPose(ignition::math::Pose3d & _pose)
{
    ignition::math::Vector3d pos = _pose.Pos();
    ignition::math::Quaterniond rot = _pose.Rot();
    this->virtual_joints.at(0)->SetPosition(0, pos.X());
    this->virtual_joints.at(1)->SetPosition(0, pos.Y());
    this->virtual_joints.at(2)->SetPosition(0, pos.Z());
    this->virtual_joints.at(3)->SetPosition(0, rot.Roll());
    this->virtual_joints.at(4)->SetPosition(0, rot.Pitch());
    this->virtual_joints.at(5)->SetPosition(0, rot.Yaw());

    this->resetJoints();
    this->imobilise();
}

/////////////////////////////////////////////////
void HandPlugin::setVelocity(std::vector<double> & _velocity)
{
    for (int i = 0; i < _velocity.size(); i++) {
        this->virtual_joints.at(i)->SetVelocity(0, _velocity.at(i));
    }
}

/////////////////////////////////////////////////
void HandPlugin::setJointVelocities(std::vector<double> & _velocities)
{
    for (int i = 0; i < _velocities.size(); i++)
    {
        this->finger_joints.at(i).actuated->SetVelocity(0, _velocities.at(i));
        for (int j = 0; j < this->finger_joints.at(i).mimic.size(); j++)
        {
            physics::JointPtr mimic_joint = this->finger_joints.at(i).mimic.at(j);
            double multiplier = this->finger_joints.at(i).multipliers.at(j);
            mimic_joint->SetVelocity(0, _velocities.at(i) * multiplier);
        }
    }
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

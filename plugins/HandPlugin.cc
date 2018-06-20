/*!
    \file plugins/HandPlugin.cc
    \brief Hand Gazebo plugin

    TODO

    \author João Borrego : jsbruglie
*/

#include "HandPlugin.hh"

namespace gazebo {

/// \brief Class for joint and respective mimics data.
class JointGroup
{
    /// Actuated joint
    public: physics::JointPtr actuated;
    /// Vector of mimic joints
    public: std::vector<physics::JointPtr> mimic;
    /// Vector of corresponding mimic joint's multipliers
    public: std::vector<double> multipliers;

    public: JointGroup(physics::JointPtr joint)
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
    if (loadJointGroups(_sdf) != true) return;
    // Virtual joints for unconstrained movement
    if (loadVirtualJoints(_sdf) != true) return;
    // Load PID controllers
    if (loadControllers() != true) return;
    // Enable/disable gravity
    if (_sdf->HasElement(PARAM_GRAVITY))
    {
        bool gravity_enabled = _sdf->Get<bool>(PARAM_GRAVITY);
        this->model->SetGravityMode(gravity_enabled);
    }

    // Set default pose
    ignition::math::Pose3d init_pose(0,0,1,0,0,0);
    setPose(init_pose);

    // Connect to world update event
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&HandPlugin::onUpdate, this));

    // Setup transport node
    this->data_ptr->node = transport::NodePtr(new transport::Node());
    this->data_ptr->node->Init();
    // Subcribe to the monitored requests topic
    this->data_ptr->sub = this->data_ptr->node->Subscribe(REQUEST_TOPIC,
        &HandPlugin::onRequest, this);
    // Publish to the hand plugin topic
    this->data_ptr->pub = this->data_ptr->node->
        Advertise<HandMsg>(RESPONSE_TOPIC);

    gzmsg << "[HandPlugin] Loaded plugin." << std::endl;
}

/////////////////////////////////////////////////
bool HandPlugin::loadMimicJoints(sdf::ElementPtr _sdf, JointGroup & joint)
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

                gzdbg << "   Mimic joint " << mimic_name << " k = " << multiplier << std::endl;
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
bool HandPlugin::loadJointGroups(sdf::ElementPtr _sdf)
{
    std::string joint_name;
    int inserted = 0;

    if (_sdf->HasElement(PARAM_JOINT_GROUP))
    {
        sdf::ElementPtr joint_sdf;
        for (joint_sdf = _sdf->GetElement(PARAM_JOINT_GROUP);
            joint_sdf != NULL;
            joint_sdf = joint_sdf->GetNextElement())
        {
            if (joint_sdf->HasAttribute(PARAM_NAME))
            {
                joint_sdf->GetAttribute(PARAM_NAME)->Get<std::string>(joint_name);
                this->joint_groups.emplace_back(this->model->GetJoint(joint_name));
                gzdbg << "Actuated joint " << joint_name << std::endl;
                loadMimicJoints(joint_sdf, this->joint_groups.at(inserted++));
            }
            else if (joint_sdf->GetName() == PARAM_JOINT_GROUP)
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
bool HandPlugin::loadControllers()
{
    physics::JointControllerPtr controller = model->GetJointController();

    // Reset existing PIDs
    std::map<std::string, common::PID> vel = controller->GetVelocityPIDs();
    for (const auto &entry : vel) {
        controller->SetVelocityPID(entry.first, common::PID(0,0,0));
    }
    std::map<std::string, common::PID> pos = controller->GetPositionPIDs();
    for (const auto &entry : pos) {
        controller->SetPositionPID(entry.first, common::PID(0,0,0));
    }

    for (const auto &joint : virtual_joints)
    {
        controller->SetVelocityPID(
            joint->GetScopedName(), common::PID(2,0,0));
    }
    for (const auto &group : joint_groups)
    {
        controller->SetPositionPID(
            group.actuated->GetScopedName(), common::PID(1.5,0,0));
        controller->SetPositionTarget(group.actuated->GetScopedName(), 0);
        for (const auto &joint : group.mimic)
        {
            controller->SetPositionPID(
                joint->GetScopedName(), common::PID(1.5,0,0));
            controller->SetPositionTarget(joint->GetScopedName(), 0);
        }
    }

    // DEBUG
    std::map<std::string, common::PID> new_vel = controller->GetVelocityPIDs();
    std::map<std::string, common::PID> new_pos = controller->GetPositionPIDs();
    gzdbg << "Position Controllers:" << std::endl;
    for (const auto &entry : new_pos) {
        gzdbg << "   " << entry.first
            << " P: " << entry.second.GetPGain()
            << " I: " << entry.second.GetIGain()
            << " D: " << entry.second.GetDGain()
            << "\n";
    }
    gzdbg << "Velocity Controllers:" << std::endl;
    for (const auto &entry : new_vel) {
        gzdbg << "   " << entry.first
            << " P: " << entry.second.GetPGain()
            << " I: " << entry.second.GetIGain()
            << " D: " << entry.second.GetDGain()
            << "\n";
    }

    return true;
}

/////////////////////////////////////////////////
void HandPlugin::onUpdate()
{
    std::lock_guard<std::mutex> lock(this->data_ptr->mutex);

    if (update_joint_velocities) {
        setJointVelocities(new_joint_velocities);
        update_joint_velocities = false;
    }
    if (update_pose) {
        setPose(new_pose);
        update_pose = false;
    }
    if (update_velocity) {
        setVelocity(new_velocity);
        update_velocity = false;
    }
    if (timer_active) {
        if (timeout <= world->SimTime()) {
            sendTimeout();
            timer_active = false;
        }
    }
    if (reset) {
        imobilise();
        resetWorld();
        reset = false;
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
        if (_msg->joint_velocities_size() == this->joint_groups.size())
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
    // Handle timer
    if (_msg->has_timeout()) {
        if (_msg->timeout() > 0)
        {
            common::Time duration(_msg->timeout());
            common::Time now = this->world->SimTime();
            // Set timeout for now + duration seconds
            std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
            this->timeout = now + duration;
            this->timer_active = true;
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
    physics::Link_V links = model->GetLinks();
    for (auto link : links) { link->ResetPhysicsStates(); }
    physics::Joint_V joints = model->GetJoints();
    for (auto joint : joints) { joint->SetVelocity(0, 0); }
    model->GetJointController()->Reset();

    new_velocity.clear();
    new_joint_velocities.clear();
    update_velocity = false;
    update_joint_velocities = false;
}

/////////////////////////////////////////////////
void HandPlugin::resetJoints()
{
    for (const auto &group : joint_groups) {
        group.actuated->SetPosition(0, 0);
        for (const auto &mimic_joint : group.mimic) {
            mimic_joint->SetPosition(0, 0);
        }
    }
}

/////////////////////////////////////////////////
void HandPlugin::setPose(ignition::math::Pose3d & _pose)
{
    ignition::math::Vector3d pos = _pose.Pos();
    ignition::math::Quaterniond rot = _pose.Rot();
    virtual_joints.at(0)->SetPosition(0, pos.X());
    virtual_joints.at(1)->SetPosition(0, pos.Y());
    virtual_joints.at(2)->SetPosition(0, pos.Z());
    virtual_joints.at(3)->SetPosition(0, rot.Roll());
    virtual_joints.at(4)->SetPosition(0, rot.Pitch());
    virtual_joints.at(5)->SetPosition(0, rot.Yaw());

    resetJoints();
    imobilise();
}

/////////////////////////////////////////////////
void HandPlugin::setVelocity(std::vector<double> & _velocity)
{
    physics::JointControllerPtr controller = model->GetJointController();

    for (int i = 0; i < _velocity.size(); i++) {
        controller->SetVelocityTarget(
            virtual_joints.at(i)->GetScopedName(), _velocity.at(i));
    }
}

/////////////////////////////////////////////////
void HandPlugin::setJointVelocities(std::vector<double> & _velocities)
{
    physics::JointControllerPtr controller = model->GetJointController();

    for (int i = 0; i < _velocities.size(); i++)
    {
        controller->SetPositionTarget(
            joint_groups.at(i).actuated->GetScopedName(), _velocities.at(i));
        for (int j = 0; j < this->joint_groups.at(i).mimic.size(); j++)
        {
            controller->SetPositionTarget(
                joint_groups.at(i).mimic.at(j)->GetScopedName(),
                _velocities.at(i) * joint_groups.at(i).multipliers.at(j));
        }
    }
}

/////////////////////////////////////////////////
void HandPlugin::sendTimeout()
{
    HandMsg msg;
    msg.set_timeout(this->timeout.Double());
    this->data_ptr->pub->Publish(msg);
    model->GetJointController()->Reset();
}

/////////////////////////////////////////////////
void HandPlugin::resetWorld()
{
    world->SetPhysicsEnabled(false);
    world->Reset();
    world->SetPhysicsEnabled(true);
}

}

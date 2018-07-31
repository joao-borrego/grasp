/*!
    \file plugins/DRPlugin.cc
    \brief Domain Randomization Gazebo plugin

    Plugin for handling Domain Randomization requests

    \author João Borrego : jsbruglie
*/

#include "DRPlugin.hh"

namespace gazebo {

/// \brief Class for private Domain Randomization plugin data.
class DRPluginPrivate
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
GZ_REGISTER_WORLD_PLUGIN(DRPlugin)

/////////////////////////////////////////////////
DRPlugin::DRPlugin() : WorldPlugin(),
    data_ptr(new DRPluginPrivate)
{
    gzmsg << "[DRPlugin] Launched plugin." << std::endl;
}

/////////////////////////////////////////////////
DRPlugin::~DRPlugin()
{
    gzmsg << "[DRPlugin] Unloaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void DRPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_world, "World pointer is NULL");
    this->world = _world;
    this->physics_engine = world->Physics();

    gzmsg << "[DRPlugin] Loaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void DRPlugin::onUpdate()
{
    // TODO
}

/////////////////////////////////////////////////
void DRPlugin::setGravity(const ignition::math::Vector3d & gravity)
{
    physics_engine->SetGravity(gravity);
}

/////////////////////////////////////////////////
void DRPlugin::setScale(ModelMsgPtr & msg)
{
    std::string model_name;
    ignition::math::Vector3d scale;
    physics::ModelPtr model;

    GZ_ASSERT(msg, "Invalid model message");
    GZ_ASSERT(msg->has_scale(), "Message has no scale field");
    model_name = msg->name();
    scale = msgs::ConvertIgn(msg->scale());
    model = world->ModelByName(model_name);
    GZ_ASSERT(model, "Model not found");
    model->SetScale(scale);
}

/////////////////////////////////////////////////
void DRPlugin::processInertial(
    physics::LinkPtr link,
    InertialMsgPtr & msg)
{
    physics::InertialPtr inertial;
    double mass;
    double ixx, ixy, ixz, iyy, iyz, izz;

    GZ_ASSERT(link, "Invalid link");
    inertial = link->GetInertial();
    GZ_ASSERT(msg, "Invalid inertial message");

    if (msg->has_mass())
    {
        mass = msg->mass();
        inertial->SetMass(mass);
    }
    if (msg->has_ixx() && msg->has_ixy() && 
        msg->has_ixz() && msg->has_iyy() && 
        msg->has_iyz() && msg->has_izz())
    {
        ixx = msg->ixx(); ixy = msg->ixy();
        ixz = msg->ixz(); iyy = msg->iyy();
        iyz = msg->iyz(); izz = msg->izz();
        inertial->SetInertiaMatrix(ixx, iyy, izz, ixy, ixz, iyz);
    }
}

/////////////////////////////////////////////////
void DRPlugin::processSurface(
    physics::CollisionPtr collision,
    SurfaceMsgPtr & msg)
{
    physics::SurfaceParamsPtr surface;

    GZ_ASSERT(collision, "Invalid collision");
    surface = collision->GetSurface();
    GZ_ASSERT(msg, "Invalid surface message");
    surface->ProcessMsg(*msg);
}

/////////////////////////////////////////////////
void DRPlugin::processJoint(
    physics::ModelPtr model,
    JointMsgPtr & msg)
{
    std::string joint_name;
    physics::JointPtr joint;
    msgs::Axis axis_msg;
    double value;

    GZ_ASSERT(model, "Invalid model");
    GZ_ASSERT(msg, "Invalid joint message");
    joint_name = msg->name();
    joint = model->GetJoint(joint_name);
    GZ_ASSERT(joint, "Joint not found");
    
    // axis2 is not yet used by Gazebo
    if (msg->has_axis1())
    {
        // Since every field is required,
        // filter out unwanted fields by checking for INFINITY
        axis_msg = msg->axis1();
        // Joint lower limit
        value = axis_msg.limit_lower();
        if (value != INFINITY) { joint->SetLowerLimit(0, value); }
        // Joint upper limit
        value = axis_msg.limit_upper();
        if (value != INFINITY) { joint->SetUpperLimit(0, value); }
        // Joint effort limit
        value = axis_msg.limit_effort();
        if (value != INFINITY) { joint->SetEffortLimit(0, value); }
        // Joint velocity limit
        value = axis_msg.limit_velocity();
        if (value != INFINITY) { joint->SetVelocityLimit(0, value); }
        // Joint physical velocity dependent on viscous damping coefficient
        value = axis_msg.damping();
        if (value != INFINITY) { joint->SetDamping(0, value); }
        // Joint static friction
        value = axis_msg.friction();
        if (value != INFINITY) { joint->SetParam("friction", 0, value); }
    }
    // ODE-specific parameters are not evaluated
}

/////////////////////////////////////////////////
void DRPlugin::processJointCmd(
    physics::ModelPtr model,
    JointCmdMsgPtr & msg)
{
    // Joint scoped name
    std::string joint_name;
    physics::JointPtr joint;
    physics::JointControllerPtr controller;
    double value;

    GZ_ASSERT(model, "Invalid model");
    GZ_ASSERT(msg, "Invalid Joint Command message");
    controller = model->GetJointController();
    joint_name = msg->name();

    if (msg->has_position())
    {
        processPID(POSITION, controller, joint_name, msg->position());
    }
    if (msg->has_velocity())
    {
        processPID(VELOCITY, controller, joint_name, msg->velocity());
    }
}

/////////////////////////////////////////////////
void DRPlugin::processPID(
    int type,
    physics::JointControllerPtr controller,
    const std::string & joint,
    const msgs::PID & msg)
{
    double value {0.0};
    bool requires_search {false};
    common::PID pid;
    std::map<std::string,common::PID> pids;   

    requires_search = msg.has_p_gain() || msg.has_i_gain() ||
        msg.has_d_gain() || msg.has_i_max() || msg.has_i_min();

    if (type == POSITION)
    {
        if (requires_search)
        {
            pids = controller->GetPositionPIDs();
            pid = pids[joint];
        }
        if (msg.has_target())
        {
            controller->SetPositionTarget(joint, msg.target());
        }
    }
    else if (type == VELOCITY)
    {
        if (requires_search)
        {
            pids = controller->GetVelocityPIDs();
            pid = pids[joint];
        }
        if (msg.has_target())
        {
            controller->SetVelocityTarget(joint, msg.target());
        }
    }

    if (msg.has_p_gain()) {
        pid.SetPGain(msg.p_gain());
    }
    if (msg.has_i_gain()) {
        pid.SetIGain(msg.i_gain());
    }
    if (msg.has_d_gain()) {
        pid.SetDGain(msg.d_gain());
    }
    if (msg.has_i_max()) {
        pid.SetIMax(msg.i_max());
    }
    if (msg.has_i_min()) {
        pid.SetIMin(msg.i_min());
    }
}

}

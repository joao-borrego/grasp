/*!
    \file utils/DRInterface.cc
    \brief Domain randomization interface

    \author João Borrego : jsbruglie
*/

#include "DRInterface.hh"

// Constants
const char DRInterface::REQUEST_TOPIC[]  = "~/dr";
const char DRInterface::RESPONSE_TOPIC[] = "~/dr/response";
const int  DRInterface::POSITION = 0;
const int  DRInterface::VELOCITY = 1;

//////////////////////////////////////////////////
DRInterface::DRInterface()
{
    node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node->Init();
    pub = node->Advertise<DRRequest>(REQUEST_TOPIC);
    pub->WaitForConnection();
}

//////////////////////////////////////////////////
DRRequest DRInterface::createRequest()
{
    DRRequest msg;
    return msg;
}

//////////////////////////////////////////////////
void DRInterface::publish(const DRRequest & msg)
{
    pub->Publish(msg);
}

//////////////////////////////////////////////////
void DRInterface::addGravity(DRRequest & msg,
    const ignition::math::Vector3d & gravity)
{
    gazebo::msgs::Physics *physics_msg;
    gazebo::msgs::Vector3d *gravity_msg;

    physics_msg = msg.mutable_physics();
    gravity_msg = physics_msg->mutable_gravity();
    gazebo::msgs::Set(gravity_msg, gravity);
}

//////////////////////////////////////////////////
void DRInterface::addModelScale(DRRequest & msg,
    const std::string & model,
    const ignition::math::Vector3d & scale)
{
    gazebo::msgs::Model *model_msg;
    gazebo::msgs::Vector3d *scale_msg;

    model_msg = msg.add_model();
    model_msg->set_name(model);
    scale_msg = model_msg->mutable_scale();
    gazebo::msgs::Set(scale_msg, scale);
}

//////////////////////////////////////////////////
void DRInterface::addLinkMass(DRRequest & msg,
    const std::string & model,
    const std::string & link,
    double mass)
{
    gazebo::msgs::Model *model_msg;
    gazebo::msgs::Link *link_msg;
    gazebo::msgs::Inertial *inertial_msg;

    model_msg = msg.add_model();
    model_msg->set_name(model);
    link_msg = model_msg->add_link();
    link_msg->set_name(link);
    inertial_msg = link_msg->mutable_inertial();
    inertial_msg->set_mass(mass);
}

//////////////////////////////////////////////////
void DRInterface::addInertiaMatrix(DRRequest & msg,
    const std::string & model,
    const std::string & link,
    double ixx, double iyy, double izz,
    double ixy, double ixz, double iyz)
{
    gazebo::msgs::Model *model_msg;
    gazebo::msgs::Link *link_msg;
    gazebo::msgs::Inertial *inertial_msg;

    model_msg = msg.add_model();
    model_msg->set_name(model);
    link_msg = model_msg->add_link();
    link_msg->set_name(link);
    inertial_msg = link_msg->mutable_inertial();
    inertial_msg->set_ixx(ixx);
    inertial_msg->set_iyy(iyy);
    inertial_msg->set_izz(izz);
    inertial_msg->set_ixy(ixy);
    inertial_msg->set_ixz(ixz);
    inertial_msg->set_iyz(iyz);
}

//////////////////////////////////////////////////
void DRInterface::addSurface(DRRequest & msg,
    const std::string & model,
    const std::string & link,
    const std::string & collision,
    const gazebo::msgs::Surface & surface)
{
    gazebo::msgs::Model *model_msg;
    gazebo::msgs::Link *link_msg;
    gazebo::msgs::Collision *collision_msg;
    gazebo::msgs::Surface *surface_msg;

    model_msg = msg.add_model();
    model_msg->set_name(model);
    link_msg = model_msg->add_link();
    link_msg->set_name(link);
    collision_msg = link_msg->add_collision();
    collision_msg->set_name(collision);
    surface_msg = collision_msg->mutable_surface();
    *surface_msg = surface;
}

//////////////////////////////////////////////////
void DRInterface::addJoint(DRRequest & msg,
    const std::string & model,
    const std::string & joint,
    double limit_lower,
    double limit_upper,
    double limit_effort,
    double limit_velocity,
    double damping,
    double friction)
{
    gazebo::msgs::Model *model_msg;
    gazebo::msgs::Joint *joint_msg;
    gazebo::msgs::Axis *axis_msg;

    model_msg = msg.add_model();
    model_msg->set_name(model);
    joint_msg = model_msg->add_joint();
    joint_msg->set_name(joint);
    axis_msg = joint_msg->mutable_axis1();
    axis_msg->set_limit_lower(limit_lower);
    axis_msg->set_limit_upper(limit_upper);
    axis_msg->set_limit_effort(limit_effort);
    axis_msg->set_limit_velocity(limit_velocity);
    axis_msg->set_damping(damping);
    axis_msg->set_friction(friction);

    // Required fields - set to dummy unused values
    ignition::math::Vector3d xyz(0,0,0);
    gazebo::msgs::Vector3d *xyz_msg;
    xyz_msg = axis_msg->mutable_xyz();
    gazebo::msgs::Set(xyz_msg, xyz);
    axis_msg->set_use_parent_model_frame(true);
}

//////////////////////////////////////////////////
void DRInterface::addModelCmd(DRRequest & msg,
    const std::string & model,
    const std::string & joint,
    int type,
    double p_gain,
    double i_gain,
    double d_gain)
{
    ModelCmdMsg *model_cmd;
    gazebo::msgs::JointCmd *joint_cmd;
    gazebo::msgs::PID *pid;

    if (type != POSITION && type != VELOCITY) { return; }

    model_cmd = msg.add_model_cmd();
    model_cmd->set_model_name(model);
    joint_cmd = model_cmd->add_joint_cmd();
    joint_cmd->set_name(joint);

    if (type == POSITION) { pid = joint_cmd->mutable_position(); }
    else                  { pid = joint_cmd->mutable_velocity(); }

    if (p_gain != INFINITY) { pid->set_p_gain(p_gain); }
    if (i_gain != INFINITY) { pid->set_i_gain(i_gain); }
    if (d_gain != INFINITY) { pid->set_d_gain(d_gain); }
}

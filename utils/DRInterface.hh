/*!
    \file utils/DRInterface.hh
    \brief Domain randomization interface headers

    \author João Borrego : jsbruglie
*/

#ifndef _DOMAIN_RANDOMIZATION_INTERFACE_HH_
#define _DOMAIN_RANDOMIZATION_INTERFACE_HH_

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/physics/bullet/BulletSurfaceParams.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>
#include <gazebo/physics/dart/DARTSurfaceParams.hh>

// Custom messages
#include "dr_request.pb.h"
#include "model_cmd.pb.h"

// Custom utilities
#include "utils.hh"
// Debug streams
#include "debug.hh"

// Required fields workaround
#include <limits>

/// Declaration for request message type
typedef grasp::msgs::DRRequest DRRequest;
/// Declaration for model command message type
typedef grasp::msgs::ModelCmd ModelCmdMsg;

/*
typedef boost::shared_ptr <gazebo::physics::BulletSurfaceParams>
    BulletSurfaceParamsPtr;
typedef boost::shared_ptr <gazebo::physics::DARTSurfaceParams>
    DARTSurfaceParamsPtr;
typedef boost::shared_ptr <gazebo::physics::ODESurfaceParams>
    ODESurfaceParamsPtr;
*/

/// \brief Domain Randomization Plugin Interface
class DRInterface
{

    /// Topic for incoming requests
    public: static const char REQUEST_TOPIC[];
    /// Topic for outgoing responses
    public: static const char RESPONSE_TOPIC[];
    /// Position controller type
    public: static const int POSITION;
    /// Velocity controller type
    public: static const int VELOCITY;

    /// Node used for transport
    private: gazebo::transport::NodePtr node;
    /// Publisher to the request topic
    private: gazebo::transport::PublisherPtr pub;

    /// \brief Constructor
    public: DRInterface();

    /// \brief Creates a domain randomization request
    /// \return Empty request
    public: DRRequest createRequest();

    /// \brief Publishes request
    /// \param Domain randomization request
    public: void publish(const DRRequest & msg);    

    // Features

    /// \brief Updates physics engine gravity
    /// \param msg Output domain randomization request
    /// \param gravity New gravity vector
    public: void addGravity(DRRequest & msg,
        const ignition::math::Vector3d & gravity);

    /// \brief Updates model scale
    /// \param msg Output domain randomization request
    /// \param model The target model name
    /// \param scale The new model scale
    public: void addModelScale(DRRequest & msg,
        const std::string & model,
        const ignition::math::Vector3d & scale);

    /// \brief Updates link mass
    /// \param msg Output domain randomization request
    /// \param model The target model name
    /// \param link The target link name
    /// \param mass The new link mass
    public: void addLinkMass(DRRequest & msg,
        const std::string & model,
        const std::string & link,
        double mass);

    /// \brief Updates link inertia matrix
    /// \param msg Output domain randomization request
    /// \param model The target model name
    /// \param link The target link name
    /// \param ixx The new Ixx
    /// \param iyy The new Iyy
    /// \param izz The new Izz
    /// \param ixy The new Ixy
    /// \param ixz The new Ixz
    /// \param iyz The new Iyz
    public: void addInertiaMatrix(DRRequest & msg,
        const std::string & model,
        const std::string & link,
        double ixx, double iyy, double izz,
        double ixy, double ixz, double iyz);

    /// \brief Updates collision surface
    /// \param msg Output domain randomization request
    /// \param model The target model
    /// \param link The target link name
    /// \param collision The target collision name
    /// \param surface The new surface parameters
    public: void addSurface(DRRequest & msg,
        const std::string & model,
        const std::string & link,
        const std::string & collision,
        const gazebo::msgs::Surface & surface);

    /// \brief Updates joint
    /// 
    /// \note Does not update value if it is INFINITY
    ///
    /// \param msg Output domain randomization request
    /// \param model The target model name
    /// \param joint The target joint name
    /// \param limit_lower New joint lower limit
    /// \param limit_upper New joint upper limit
    /// \param limit_effort New joint effort limit
    /// \param limit_velocity New joint velocity limit
    /// \param limit_lower New joint damping coefficient (TODO)
    /// \param limit_lower New joint static friction
    public: void addJoint(DRRequest & msg,
        const std::string & model,
        const std::string & joint,
        double limit_lower = INFINITY,
        double limit_upper = INFINITY,
        double limit_effort = INFINITY,
        double limit_velocity = INFINITY,
        double damping = INFINITY,
        double friction = INFINITY);

    /// \brief Updates model controllers
    /// 
    /// \note Does not update value if it is INFINITY
    ///
    /// \param msg Output domain randomization request
    /// \param model The target model name
    /// \param joint The target joint <b>scoped</b> name
    /// \param type Type of controllers (POSITION or VELOCITY)
    /// \param p_gain New P gain
    /// \param i_gain New I gain 
    /// \param d_gain New D gain 
    public: void addModelCmd(DRRequest & msg,
        const std::string & model,
        const std::string & joint,
        int type,
        double p_gain = INFINITY,
        double i_gain = INFINITY,
        double d_gain = INFINITY);
};

#endif

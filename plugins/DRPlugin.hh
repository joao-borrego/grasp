/*!
    \file plugins/DRPlugin.hh
    \brief Domain Randomization Gazebo plugin

    Plugin for handling Domain Randomization requests

    \author João Borrego : jsbruglie
*/

#ifndef _DOMAIN_RANDOMIZATION_PLUGIN_HH_
#define _DOMAIN_RANDOMIZATION_PLUGIN_HH_

// Gazebo
#include "gazebo/common/Plugin.hh"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/sensors.hh>

#include <limits>

namespace DRPlugin {

    // Plugin messages

    /// Topic for incoming requests
    #define REQUEST_TOPIC   "~/dr"
     /// Topic for outgoing responses
    #define RESPONSE_TOPIC  "~/dr/response"

    /// Position controller type
    #define POSITION (int) 0
    /// Velocity controller type
    #define VELOCITY (int) 1
}

namespace gazebo {

    // Message types

    /// Model message pointer
    typedef const boost::shared_ptr<const msgs::Model> ModelMsgPtr;
    /// Link message pointer
    typedef const boost::shared_ptr<const msgs::Link> LinkMsgPtr;
    /// Inertial message pointer
    typedef const boost::shared_ptr<const msgs::Inertial> InertialMsgPtr;
    /// Surface message pointer
    typedef const boost::shared_ptr<const msgs::Surface> SurfaceMsgPtr;
    /// Joint message pointer
    typedef const boost::shared_ptr<const msgs::Joint> JointMsgPtr;
    /// Joint command message pointer
    typedef const boost::shared_ptr<const msgs::JointCmd> JointCmdMsgPtr;
    /// PID controller message pointer
    typedef const boost::shared_ptr<const msgs::PID> PIDMsgPtr;

    // Forward declaration of private data class
    class DRPluginPrivate;

    // TODO
    class DRPlugin : public WorldPlugin
    {
        // Private attributes

        /// Class with private attributes
        private: std::unique_ptr<DRPluginPrivate> data_ptr;
        /// Connection to world update event
        private: event::ConnectionPtr update_connection;

        /// World to which the plugin is attached
        private: physics::WorldPtr world;
        /// Physics engine pointer
        private: physics::PhysicsEnginePtr physics_engine;

        // Public methods

        /// \brief Constructs the object
        public: DRPlugin();

        /// \brief Destroys the object
        public: virtual ~DRPlugin();

        /// \brief Loads the plugin
        /// \param _world The world pointer
        /// \param _sdf   The sdf element pointer
        public: virtual void Load(
            physics::WorldPtr _world, sdf::ElementPtr _sdf);

        /// \brief Called on World Update event
        public: void onUpdate();

        // Physics 

        /// \brief Updates gravity
        /// \param gravity New gravity vector
        private: void setGravity(const ignition::math::Vector3d & gravity);

        /// \brief Updates model scale
        /// \param msg Model message pointer
        private: void setScale(ModelMsgPtr & msg);

        /// \brief Updates inertial
        /// \param link Parent link pointer
        /// \param msg Inertial message pointer
        private: void processInertial(
            physics::LinkPtr link,
            InertialMsgPtr & msg);

        /// \brief Updates surface
        /// \param collision Parent collision pointer
        /// \param msg Surface message pointer
        private: void processSurface(
            physics::CollisionPtr collision,
            SurfaceMsgPtr & msg);

        /// \brief Updates joint
        /// \param model Parent model pointer
        /// \param msg Joint message pointer
        private: void processJoint(
            physics::ModelPtr model,
            JointMsgPtr & msg);

        /// \brief Processes joint command message
        /// \param model Parent model pointer
        /// \param msg Joint command message pointer
        private: void processJointCmd(
            physics::ModelPtr model,
            JointCmdMsgPtr & msg);
        
        /// \brief Updates PID controller
        /// \param type PID controller type (POSITION or VELOCITY)
        /// \param controller Joint controller pointer
        /// \param msg PID message pointer
        private: void processPID(
            int type,
            physics::JointControllerPtr controller,
            const std::string & joint,
            const msgs::PID & msg);

    };
}

#endif

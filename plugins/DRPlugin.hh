/*!
    \file plugins/DRPlugin.hh
    \brief Domain randomization Gazebo plugin headers

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

// Custom messages
#include "dr_request.pb.h"
#include "model_cmd.pb.h"

// Required fields workaround
#include <limits>

// TODO - Migrate
#define NULL_CHECK(_expr, _msg) if (!_expr) \
{                                           \
    gzdbg << _msg << std::endl;             \
    return;                                 \
}

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

    /// Declaration for request message type
    typedef grasp::msgs::DRRequest DRRequest;
    /// Shared pointer declaration for request message type
    typedef const boost::shared_ptr<const grasp::msgs::DRRequest>
        DRRequestPtr;
    /// Declaration for model command message type
    typedef grasp::msgs::ModelCmd ModelCmdMsg;
    /// Shared pointer declaration for model command message type
    typedef const boost::shared_ptr<const grasp::msgs::ModelCmd>
        ModelCmdPtr;
    
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

        /// Pending request
        private: boost::shared_ptr<DRRequest const> msg;

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

        /// \brief Callback function for handling incoming requests
        /// \param _msg  The message
        public: void onRequest(DRRequestPtr & _msg);

        // Physics 

        /// \brief Processes physics message
        /// \param msg Physics message
        private: void processPhysics(const msgs::Physics & msg);

        /// \brief Processes model message
        /// \param msg Model message
        private: void processModel(const msgs::Model & msg);

        /// \brief Updates joint
        /// \param model Parent model pointer
        /// \param msg Joint message
        private: void processJoint(
            physics::ModelPtr model,
            const msgs::Joint & msg);

        /// \brief Updates link
        /// \param model Parent model pointer
        /// \param msg Link message
        private: void processLink(
            physics::ModelPtr model,
            const msgs::Link & msg);

        /// \brief Updates inertial
        /// \param link Parent link pointer
        /// \param msg Inertial message
        private: void processInertial(
            physics::LinkPtr link,
            const msgs::Inertial & msg);

        /// \brief Updates surface
        /// \param collision Parent collision pointer
        /// \param msg Surface message
        private: void processSurface(
            physics::CollisionPtr collision,
            const msgs::Surface & msg);

        /// \brief Processes model command message
        /// \param model Parent model pointer
        /// \param msg Joint command message pointer
        private: void processModelCmd(
            const ModelCmdMsg & msg);

        /// \brief Processes joint command message
        /// \param model Parent model pointer
        /// \param msg Joint command message pointer
        private: void processJointCmd(
            physics::ModelPtr model,
            const msgs::JointCmd & msg);
        
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

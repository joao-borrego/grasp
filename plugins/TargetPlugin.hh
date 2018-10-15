/*!
    \file plugins/TargetPlugin.hh
    \brief Grasp target Gazebo plugin

    // TODO

    \author João Borrego : jsbruglie
*/

#ifndef _TARGET_PLUGIN_HH_
#define _TARGET_PLUGIN_HH_

// Gazebo
#include "gazebo/common/Plugin.hh"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Custom messages
#include "target_request.pb.h"
#include "target_response.pb.h"

namespace TargetPlugin {

    // Plugin messages

    /// Topic for incoming requests
    #define REQUEST_TOPIC   "~/grasp/target"
    /// Topic for outgoing responses
    #define RESPONSE_TOPIC  "~/grasp/target/response"

    /// Get pose request
    #define REQ_GET_POSE        grasp::msgs::TargetRequest::GET_POSE
    /// Set pose request
    #define REQ_SET_POSE        grasp::msgs::TargetRequest::SET_POSE
    /// Get updated resting pose request
    #define REQ_GET_REST_POSE   grasp::msgs::TargetRequest::GET_REST_POSE
    /// Reset request
    #define REQ_RESET           grasp::msgs::TargetRequest::RESET
    /// Current pose response
    #define RES_POSE            grasp::msgs::TargetResponse::POSE
    /// Updated rest pose response
    #define RES_REST_POSE       grasp::msgs::TargetResponse::REST_POSE

    /// Maximum linear velocity for an object to be considered static
    #define LIN_VEL_EPSILON 0.000001
    /// Maximum angular velocity for an object to be considered static
    #define ANG_VEL_EPSILON 0.00001
    /// Size of reachable grid
    #define GRID_SIZE 10
}

namespace gazebo {

    /// Declaration for request message type
    typedef grasp::msgs::TargetRequest TargetRequest;
    /// Shared pointer declaration for request message type
    typedef const boost::shared_ptr<const grasp::msgs::TargetRequest>
        TargetRequestPtr;
    /// Declaration for response message type
    typedef grasp::msgs::TargetResponse TargetResponse;
    /// Shared pointer declaration for response message type
    typedef const boost::shared_ptr<const grasp::msgs::TargetResponse>
        TargetResponsePtr;

    // Forward declaration of private data class
    class TargetPluginPrivate;

    /// \brief Grasp target plugin class
    class TargetPlugin : public ModelPlugin
    {
        // Private attributes

        /// Class with private attributes
        private: std::unique_ptr<TargetPluginPrivate> data_ptr;
        /// Model to which the plugin is attached
        private: physics::ModelPtr model;
        /// Connection to world update event
        private: event::ConnectionPtr update_connection;
        /// Connection to world reset event
        //private: event::ConnectionPtr reset_connection;

        /// Flag for pending get pose request
        private: bool get_pose {false};
        /// Flag for pending set pose request
        private: bool set_pose {false};
        /// Flag for pending update resting pose request
        private: bool update_rest_pose {false};
        /// Flag for pending reset
        private: bool reset {false};
        /// Initial pose
        private: ignition::math::Pose3d init_pose;
        /// New pose
        private: ignition::math::Pose3d new_pose;

        // Public methods

        /// \brief Constructs the object
        public: TargetPlugin();

        /// \brief Destroys the object
        public: virtual ~TargetPlugin() override;

        /// \brief Loads the plugin
        /// \param _model The model pointer
        /// \param _sdf   The sdf element pointer
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief Returns whether object is out of bounds
        /// \return Whether object is outside reachable grid
        private: bool outOfBounds();

        /// \brief Callback on world update event
        public: void onUpdate();

        /// \brief Callback on world reset event
        public: void onReset();

        /// \brief Callback function for handling incoming requests
        /// \param _msg  The message
        public: void onRequest(TargetRequestPtr & _msg);

    };
}

#endif

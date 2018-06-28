/*!
    \file plugins/HandPlugin.hh
    \brief Hand Gazebo plugin

    TODO

    \author João Borrego : jsbruglie
*/

#ifndef _HAND_PLUGIN_HH_
#define _HAND_PLUGIN_HH_

// Gazebo
#include "gazebo/common/Plugin.hh"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Custom messages
#include "hand.pb.h"

// Boost - for convenient string split
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

namespace HandPlugin {

    // Plugin parameters instanced in SDF

    /// Name SDF attribute
    #define PARAM_NAME              "name"
    /// Mimic joint multiplier SDF attribute
    #define PARAM_MULTIPLIER        "multiplier"
    /// Finger joint name SDF entity
    #define PARAM_JOINT_GROUP       "actuatedJoint"
    /// Mimic joint name SDF entity
    #define PARAM_MIMIC_JOINT       "mimicJoint"
    /// Virtual revolute joint name SDF entity
    #define PARAM_VIRTUAL_JOINTS    "virtualJoints"
    /// Set gravity state SDF entity
    #define PARAM_GRAVITY           "gravity"

    // Plugin messages

    /// Topic for incoming requests
    #define REQUEST_TOPIC    "~/hand"
    /// Topic for outgoing responses
    #define RESPONSE_TOPIC    "~/hand/response"
}

namespace gazebo {

    /// Declaration for request message type
    typedef grasp::msgs::Hand HandMsg;
    /// Shared pointer declaration for request message type
    typedef const boost::shared_ptr<const grasp::msgs::Hand>
        HandMsgPtr;

    // Forward declaration of private joint class
    class JointGroup;

    // Forward declaration of private data class
    class HandPluginPrivate;

    class HandPlugin : public ModelPlugin
    {
        // Private attributes

        /// Class with private attributes
        private: std::unique_ptr<HandPluginPrivate> data_ptr;
        /// Model to which the plugin is attached
        private: physics::ModelPtr model;
        /// World in which the model exists
        private: physics::WorldPtr world;
        /// Connection to world update event
        private: event::ConnectionPtr update_connection;

        /// Array of finger joints
        private: std::vector<JointGroup> joint_groups;
        /// Array of virtual x y z r p y joint pointers
        private: std::vector<physics::JointPtr> virtual_joints;

        /// Timeout trigger
        private: bool timer_active {false};
        /// Next timeout
        private: common::Time timeout;

        /// Initial pose
        private: ignition::math::Pose3d init_pose {0,0,1,0,0,0};
        /// Flag for message received
        private: bool recv_msg {false};
        /// Last received request message
        private: boost::shared_ptr<HandMsg const> msg_req;

        // Protected attributes

        // Public methods

        /// \brief Constructs the object
        public: HandPlugin();

        /// \brief Destroys the object
        public: virtual ~HandPlugin();

        /// \brief Loads the plugin
        /// \param _model The model pointer
        /// \param _sdf   The sdf element pointer
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief Updates model on world update
        public: void onUpdate();

        /// \brief Callback function for handling incoming requests
        /// \param _msg  The message
        public: void onRequest(HandMsgPtr & _msg);

        // Private methods

        /// TODO
        private: bool loadMimicJoints(sdf::ElementPtr _sdf, JointGroup & joint);

        /// \brief Loads finger joints
        /// \param _sdf The root sdf element pointer
        /// \returns Success
        private: bool loadJointGroups(sdf::ElementPtr _sdf);

        /// \brief Loads virtual joints
        /// \param _sdf The root sdf element pointer
        /// \returns Success
        private: bool loadVirtualJoints(sdf::ElementPtr _sdf);

        /// \brief Loads PID controllers for each joint
        /// \returns Success
        private: bool loadControllers();

        /// \brief Imobilises the hand
        private: void imobilise();

        /// \brief Resets finger joints to default pose
        private: void resetJoints();

        // Requests

        /// \brief Changes hand's pose
        ///
        /// Since the hand is fixed to the world, the pose can not be set
        /// directly, and instead must be obtained using the virtual
        /// joints.
        ///
        /// \param pose The new pose
        private: void setPose(const ignition::math::Pose3d & pose);

        /// \brief Changes hand's pose on request
        /// \param _msg The request message
        private: void updatePose(HandMsgPtr & _msg);

        /// \brief Updates internal timer
        /// \param _msg The request message
        private: void updateTimer(HandMsgPtr & _msg);

        /// \brief Change PID target
        /// \param type Type of PID control
        /// \param joint Scoped name of controlled joint
        /// \param value Desired target value
        private: void setPIDTarget(
            int type, const std::string & joint, double value);

        /// \brief Changes hand's pid controller targets
        /// \param _msg The request message
        private: void updatePIDTargets(HandMsgPtr & _msg);

        /// \brief Resets world on request
        /// \param _msg The request message
        private: void checkReset(HandMsgPtr & _msg);

        /// \brief Broadcasts a timeout message
        private: void sendTimeout();

        /// \brief Resets the world
        private: void resetWorld();
    };
}

#endif

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

    /// Virtual revolute joint name parameter SDF
    #define PARAM_VIRTUAL_JOINTS    "virtualJoints"
    /// Set gravity state parameter SDF
    #define PARAM_GRAVITY           "gravity"

    // Plugin messages

    /// Topic for incoming requests
    #define REQUEST_TOPIC    "~/hand"
}

namespace gazebo {

    /// Shared pointer declaration for request message type
    typedef const boost::shared_ptr<const grasp::msgs::Hand>
        HandMsgPtr;

    // Forward declaration of private data class
    class HandPluginPrivate;

    class HandPlugin : public ModelPlugin
    {
        // Private attributes

        /// Class with private attributes
        private: std::unique_ptr<HandPluginPrivate> data_ptr;
        /// Model to which the plugin is attached
        private: physics::ModelPtr model;
        /// Connection to world update event
        private: event::ConnectionPtr update_connection;

        /// Array of virtual x y z r p y joint pointers
        private: std::vector<physics::JointPtr> virtual_joints;

        /// New pose
        private: ignition::math::Pose3d new_pose;
        /// New velocity vector
        private: std::vector<double> new_velocity;
        /// Flag to update pose
        private: bool update_pose       {false};
        /// Flag to update velocity
        private: bool update_velocity   {false};
        /// Flag to reset
        private: bool reset             {false};

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

        /// \brief Imobilises the hand
        private: void imobilise();

        /// \brief Changes hand's pose
        ///
        /// Since the hand is fixed to the world, the pose can not be set
        /// directly, and instead must be obtained using the virtual
        /// joints.
        ///
        /// \param pose The hand's new pose
        private: void setPose(ignition::math::Pose3d & _pose);

        /// \brief Changes hand's velocity
        /// \param pose The hand's new velocity vector
        private: void setVelocity(std::vector<double> & _velocity);

        /// \brief Resets the world
        private: void resetWorld();
    };
}

#endif

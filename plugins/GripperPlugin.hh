/*!
    \file plugins/GripperPlugin.hh
    \brief Gripper Gazebo plugin

    TODO

    \author João Borrego : jsbruglie
*/

#ifndef _GRIPPER_PLUGIN_HH_
#define _GRIPPER_PLUGIN_HH_

// Gazebo
#include "gazebo/common/Plugin.hh"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Custom messages
#include "gripper.pb.h"

// Boost - for convenient string split
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

namespace GripperPlugin {

    // Plugin parameters instanced in SDF

    /// Base link name parameter SDF
    #define PARAM_BASE_LINK         "baseLink"
    /// Left gripper link name parameter SDF
    #define PARAM_LEFT_LINK         "leftLink"
    /// Right gripper link name parameter SDF
    #define PARAM_RIGHT_LINK        "rightLink"
    /// Left gripper joint name parameter SDF
    #define PARAM_LEFT_JOINT        "leftJoint"
    /// Right gripper joint name parameter SDF
    #define PARAM_RIGHT_JOINT       "rightJoint"
    /// Virtual revolute joint name parameter SDF
    #define PARAM_VIRTUAL_JOINTS    "virtualJoints"
    /// Set gravity state parameter SDF
    #define PARAM_GRAVITY           "gravity"

    // Plugin messages

    /// Topic for incoming requests
    #define REQUEST_TOPIC    "~/gripper"
}

namespace gazebo {

    /// Shared pointer declaration for request message type
    typedef const boost::shared_ptr<const grasp::msgs::Gripper>
        GripperMsgPtr;

    // Forward declaration of private data class
    class GripperPluginPrivate;

    class GripperPlugin : public ModelPlugin
    {
        // Private attributes

        /// Class with private attributes
        private: std::unique_ptr<GripperPluginPrivate> data_ptr;
        /// Model to which the plugin is attached
        private: physics::ModelPtr model;
        /// Connection to world update event
        private: event::ConnectionPtr update_connection;

        /// Model name
        private: std::string model_name;
        /// Base link pointer
        private: physics::LinkPtr base_link;
        /// Left gripper link pointer
        private: physics::LinkPtr left_link;
        /// Right gripper link pointer
        private: physics::LinkPtr right_link;
        /// Left gripper joint pointer
        private: physics::JointPtr left_joint;
        /// Right gripper joint pointer
        private: physics::JointPtr right_joint;
        /// Array of virtual x y z r p y joint pointers
        private: std::vector<physics::JointPtr> virtual_joints;

        /// New pose
        private: ignition::math::Pose3d new_pose;
        /// New velocity vector
        private: std::vector<double> new_velocity;
        /// New open/closed state
        private: bool new_open          {false};
        /// Flag to update pose
        private: bool update_pose       {false};
        /// Flag to update velocity
        private: bool update_velocity   {false};
        /// Current open/closed state
        private: bool open              {false};
        /// Flag to reset
        private: bool reset             {false};

        // Protected attributes

        // Public methods

        /// \brief Constructs the object
        public: GripperPlugin();

        /// \brief Destroys the object
        public: virtual ~GripperPlugin();

        /// \brief Loads the plugin
        /// \param _model The model pointer
        /// \param _sdf   The sdf element pointer
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief Updates model on world update
        public: void onUpdate();

        /// \brief Callback function for handling incoming requests
        /// \param _msg  The message
        public: void onRequest(GripperMsgPtr & _msg);

        // Private methods

        /// \brief Imobilises the gripper
        private: void imobilise();

        /// \brief Changes gripper's pose
        ///
        /// Since the gripper is fixed to the world, the pose can not be set
        /// directly, and instead must be obtained using the virtual
        /// joints.
        ///
        /// \param pose The gripper's new pose
        private: void setPose(ignition::math::Pose3d & _pose);

        /// \brief Changes gripper's velocity
        /// \param pose The gripper's new velocity vector
        private: void setVelocity(std::vector<double> & _velocity);

        /// \brief Opens the gripper
        private: void openGripper();

        /// \brief Closes the gripper
        private: void closeGripper();

        /// \brief Resets the world
        private: void resetWorld();
    };
}

#endif

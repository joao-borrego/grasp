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

namespace gazebo{

    // Forward declaration of private data class
    class GripperPluginPrivate;

    class GripperPlugin : public ModelPlugin {

        // Private attributes

        /// Class with private attributes
        private: std::unique_ptr<GripperPluginPrivate> dataPtr;

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

    };
}

#endif

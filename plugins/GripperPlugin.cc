/*!
    \file plugins/GripperPlugin.cc
    \brief Gripper Gazebo plugin

    TODO

    \author João Borrego : jsbruglie
*/

#include "GripperPlugin.hh"

namespace gazebo {


/// \brief Class for private camera utils plugin data.
class GripperPluginPrivate
{
    /// Gazebo transport node 
    public: transport::NodePtr node;
    /// Camera utils topic subscriber 
    public: transport::SubscriberPtr sub;
    /// Camera utils topic publisher 
    public: transport::PublisherPtr pub;

    /// Mutex for safe data access
    public: std::mutex mutex;
};

// Register this plugin with the simulator 
GZ_REGISTER_MODEL_PLUGIN(GripperPlugin)

/////////////////////////////////////////////////
GripperPlugin::GripperPlugin()
    : ModelPlugin(), dataPtr(new GripperPluginPrivate)
{
    gzmsg << "[GripperPlugin] Loaded gripper plugin." << std::endl;
}

/////////////////////////////////////////////////
GripperPlugin::~GripperPlugin()
{
    gzmsg << "[GripperPlugin] Unloaded gripper plugin." << std::endl;
}

/////////////////////////////////////////////////
void GripperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Safety check
    if (_model->GetJointCount() == 0)
    {
        gzerr << "[GripperPlugin] Invalid model." << std::endl;
        return;
    }
}

}

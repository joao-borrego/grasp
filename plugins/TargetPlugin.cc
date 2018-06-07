/*!
    \file plugins/TargetPlugin.cc
    \brief Grasp target Gazebo plugin

    // TODO

    \author João Borrego : jsbruglie
*/

#include "TargetPlugin.hh"

namespace gazebo {


/// \brief Class for private Target plugin data.
class TargetPluginPrivate
{
    /// Gazebo transport node
    public: transport::NodePtr node;
    /// Gazebo topic subscriber
    public: transport::SubscriberPtr sub;
    /// Gazebo topic publisher
    public: transport::PublisherPtr pub;

    /// Mutex for safe data access
    public: std::mutex mutex;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TargetPlugin)

/////////////////////////////////////////////////
TargetPlugin::TargetPlugin() : ModelPlugin(),
    data_ptr(new TargetPluginPrivate)
{
    gzmsg << "[TargetPlugin] Started plugin." << std::endl;
}

/////////////////////////////////////////////////
TargetPlugin::~TargetPlugin()
{
    gzmsg << "[TargetPlugin] Unloaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void TargetPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    this->model = _model;

    // Change world default pose to match bounding box height
    // prevents clipping through floor
    ignition::math::Pose3d pose = _model->WorldPose();
    ignition::math::Box bnd_box = _model->CollisionBoundingBox();
    ignition::math::Pose3d offset_z(0,0,bnd_box.ZLength() / 2,0,0,0);
    this->init_pose = pose + offset_z;
    _model->SetWorldPose(this->init_pose);

    // Connect to world update event
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&TargetPlugin::onUpdate, this));
    // Connect to world reset event
    this->reset_connection = event::Events::ConnectWorldReset(
        std::bind(&TargetPlugin::onReset, this));

    // Setup transport node
    this->data_ptr->node = transport::NodePtr(new transport::Node());
    this->data_ptr->node->Init();
    // Subcribe to the monitored requests topic
    this->data_ptr->sub = this->data_ptr->node->Subscribe(REQUEST_TOPIC,
        &TargetPlugin::onRequest, this);
    // Setup publisher for the response topic
    this->data_ptr->pub = this->data_ptr->node->
        Advertise<TargetResponse>(RESPONSE_TOPIC);

    gzmsg << "[TargetPlugin] Loaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void TargetPlugin::onUpdate()
{
    bool reply_pose = false;
    TargetResponse msg;
    
    {
        std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
        if (this->get_pose)
        {
            this->get_pose = false;
            reply_pose = true;
        }
        if (this->set_pose)
        {
            model->SetWorldPose(new_pose);
            set_pose = false;
            msg.set_type(RES_POSE);
            reply_pose = true;
        }
        if (this->update_rest_pose)
        {
            bool resting = true;
            double epsilon = VELOCITY_EPSILON;
            ignition::math::Vector3d lin_vel = model->WorldLinearVel(); 
            ignition::math::Vector3d ang_vel = model->WorldAngularVel();

            // Object is resting iif vel_i < epsilon, for all i
            for (int i = 0; i < 3 && resting; i++) {
                if (lin_vel[i] > epsilon) { resting = false; }
            }
            for (int i = 0; i < 3 && resting; i++) {
                if (ang_vel[i] > epsilon) { resting = false; }
            }
            if (resting)
            {
                init_pose = model->WorldPose();
                update_rest_pose = false;
                msg.set_type(RES_REST_POSE);
                reply_pose = true;
            }
        }
    }

    if (reply_pose)
    {
        gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
        gazebo::msgs::Set(pose_msg, model->WorldPose());
        msg.set_allocated_pose(pose_msg);
        this->data_ptr->pub->Publish(msg);
    }
}

/////////////////////////////////////////////////
void TargetPlugin::onReset()
{
    std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
    this->model->SetWorldPose(this->init_pose);
}

/////////////////////////////////////////////////
void TargetPlugin::onRequest(TargetRequestPtr &_msg)
{

    if (_msg->has_type())
    {
        int type = _msg->type();
        if (type == REQ_GET_POSE)
        {
            std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
            this->get_pose = true;
        }
        else if (type == REQ_SET_POSE)
        {
            std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
            this->set_pose = true;
            this->new_pose = msgs::ConvertIgn(_msg->pose());
        }
        else if (type == REQ_GET_REST_POSE)
        {
            std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
            this->update_rest_pose = true;
        }
    }
}

}

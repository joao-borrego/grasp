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
    gzmsg << "[TargetPlugin] Unloading plugin." << std::endl;
    this->update_connection.reset();
    //this->reset_connection.reset();
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
//    this->reset_connection = event::Events::ConnectWorldReset(
//        std::bind(&TargetPlugin::onReset, this));

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
    bool reset_aux = false;
    TargetResponse msg;
    double epsilon;

    {
        std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
        if (this->get_pose)
        {
            this->get_pose = false;
            msg.set_type(RES_POSE);
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
            ignition::math::Vector3d lin_vel = model->WorldLinearVel();
            ignition::math::Vector3d ang_vel = model->WorldAngularVel();

            // Object is resting iif vel_i < epsilon, for all i
            epsilon = LIN_VEL_EPSILON;
            for (int i = 0; i < 3 && resting; i++) {
                if (std::abs(lin_vel[i]) > epsilon) { resting = false; }
            }
            epsilon = ANG_VEL_EPSILON;
            for (int i = 0; i < 3 && resting; i++) {
                if (std::abs(ang_vel[i]) > epsilon) { resting = false; }
            }
            if (resting)
            {
                init_pose = model->WorldPose();
                update_rest_pose = false;
                msg.set_type(RES_REST_POSE);
                reply_pose = true;
            }
        }
        if (this->reset)
        {
            reset = false;
            msg.set_type(RES_POSE);
            reset_aux = true;
            reply_pose = true;
        }
        if (outOfBounds())
        {
            reset_aux = true;
        }
    }

    if (reply_pose)
    {
        gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
        gazebo::msgs::Set(pose_msg, model->WorldPose());
        msg.set_allocated_pose(pose_msg);
        this->data_ptr->pub->Publish(msg);
    }

    if (reset_aux)
    {
        this->onReset();
    }
}


/////////////////////////////////////////////////
bool TargetPlugin::outOfBounds()
{
    ignition::math::Pose3d pose = this->model->WorldPose();
    double x, y, z;
    x = pose.Pos().X();
    y = pose.Pos().Y();
    z = pose.Pos().Z();

    if ((-GRID_SIZE <= x && x <= GRID_SIZE) ||
        (-GRID_SIZE <= y && y <= GRID_SIZE) ||
        (-GRID_SIZE <= z && z <= GRID_SIZE))
    {
        return false;
    }
    return true;
}

/////////////////////////////////////////////////
void TargetPlugin::onReset()
{
    this->model->SetWorldPose(this->init_pose);
    this->model->SetAngularVel({0, 0, 0});
    this->model->SetLinearVel({0, 0, 0});
    gzdbg << "Model reset to pose " << init_pose << std::endl;
}

/////////////////////////////////////////////////
void TargetPlugin::onRequest(TargetRequestPtr &_msg)
{
    if (_msg->has_type())
    {
        int type = _msg->type();
        std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
        if (type == REQ_GET_POSE)
        {
            this->get_pose = true;
        }
        else if (type == REQ_SET_POSE)
        {
            this->set_pose = true;
            this->new_pose = msgs::ConvertIgn(_msg->pose());
        }
        else if (type == REQ_GET_REST_POSE)
        {
            this->update_rest_pose = true;
        }
        else if (type == REQ_RESET)
        {
            this->reset = true;
        }
    }
}

}

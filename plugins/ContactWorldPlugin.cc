/*!
    \file plugins/ContactWorldPlugin.cc
    \brief Contact manager Gazebo world plugin

    // TODO

    \author João Borrego : jsbruglie
*/

#include "ContactWorldPlugin.hh"

namespace gazebo {


/// \brief Class for private Target plugin data.
class ContactWorldPluginPrivate
{
    /// Gazebo transport node
    public: transport::NodePtr node;
    /// Gazebo request topic subscriber
    public: transport::SubscriberPtr sub_req;
    /// Gazebo contacts topic subscriber
    public: transport::SubscriberPtr sub_con;
    /// Gazebo response topic subscriber
    public: transport::SubscriberPtr sub_res;
    /// Gazebo topic publisher
    public: transport::PublisherPtr pub;

    /// Mutex for safe data access
    public: std::mutex mutex;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ContactWorldPlugin)

/////////////////////////////////////////////////
ContactWorldPlugin::ContactWorldPlugin() : WorldPlugin(),
    data_ptr(new ContactWorldPluginPrivate)
{
    gzmsg << "[ContactWorldPlugin] Started plugin." << std::endl;
}

/////////////////////////////////////////////////
ContactWorldPlugin::~ContactWorldPlugin()
{
    gzmsg << "[ContactWorldPlugin] Unloaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void ContactWorldPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
    world = _world;

    // Communications - TODO
    std::string req_topic = REQUEST_TOPIC;
    std::string res_topic = RESPONSE_TOPIC;

    // Setup transport node
    data_ptr->node = transport::NodePtr(new transport::Node());
    data_ptr->node->Init();
    // Subcribe to the monitored requests topic
    data_ptr->sub_req = data_ptr->node->Subscribe(req_topic,
        &ContactWorldPlugin::onRequest, this);
    // Subscribe to the response topic (Workaround)
    data_ptr->sub_res = data_ptr->node->Subscribe(res_topic,
        &ContactWorldPlugin::onResponse, this);
    // Publish to the response topic
    data_ptr->pub = data_ptr->node->Advertise<ContactResponse>(res_topic);

    // Connect to world update event
    update_connection = event::Events::ConnectWorldUpdateEnd(
        std::bind(&ContactWorldPlugin::onUpdate, this));

    gzmsg << "[ContactWorldPlugin] Loaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void ContactWorldPlugin::onUpdate()
{
    std::lock_guard<std::mutex> lock(data_ptr->mutex);

    // Process received message
    if (recv_msg)
    {
        gzdbg << "Processing message" << std::endl;

        // Create response message
        ContactResponse msg;
        msg.set_success(true);

        // Check for collisions between entities
        checkCollision(msg);
        // Handle change surface properties request
        changeSurface(msg);

        // Publish response
        data_ptr->pub->Publish(msg);

        // Cleanup and accept new requests
        msg_req.reset();
        enabled = false;
        recv_msg = false;
    }
}

/////////////////////////////////////////////////
void ContactWorldPlugin::onContact(ConstContactsPtr & _msg)
{
    std::lock_guard<std::mutex> lock(data_ptr->mutex);

    if (enabled) {
        recv_msg = true;
    }
}

/////////////////////////////////////////////////
void ContactWorldPlugin::onRequest(ContactRequestPtr & _msg)
{
    gzdbg << "Received message" << std::endl;

    std::lock_guard<std::mutex> lock(data_ptr->mutex);

    if (!data_ptr->sub_con) {
        // Subcribe to the monitored contacts topic
        data_ptr->sub_con = data_ptr->node->Subscribe("~/physics/contacts",
            &ContactWorldPlugin::onContact, this);
    }

    if (!msg_req)
    {
        msg_req = _msg;
        enabled = true;
    }
}

/////////////////////////////////////////////////
void ContactWorldPlugin::onResponse(ContactResponsePtr & _msg)
{
    std::lock_guard<std::mutex> lock(data_ptr->mutex);

    if (!enabled && !recv_msg) {
        data_ptr->sub_con.reset();
    }
}

/////////////////////////////////////////////////
void ContactWorldPlugin::checkCollision(ContactResponse & _msg)
{
    if (msg_req)
    {
        physics::ContactManager *manager =
            world->Physics()->GetContactManager();
        unsigned int num_contacts = manager->GetContactCount();

        for (auto & pair : msg_req->collision())
        {
            std::string col1(pair.collision1());
            std::string col2(pair.collision2());

            gzdbg << "Test " << col1 << " and " << col2 << std::endl;

            for (int i = 0; i < num_contacts; i++)
            {
                physics::Contact *contact = manager->GetContact(i);
                std::string tmp_col1(contact->collision1->GetScopedName());
                std::string tmp_col2(contact->collision2->GetScopedName());

                if ((strstr(tmp_col1.c_str(), col1.c_str()) &&
                        strstr(tmp_col2.c_str(), col2.c_str())) ||
                    (strstr(tmp_col2.c_str(), col1.c_str()) &&
                        strstr(tmp_col1.c_str(), col2.c_str())))
                {
                    gzdbg << "Contact: " << col1 <<
                        " and " << col2 << std::endl;
                    gazebo::msgs::Contact *contact = _msg.add_contacts();
                    gazebo::msgs::Time *time = new gazebo::msgs::Time();
                    time->set_sec(world->RealTime().sec);
                    time->set_nsec(world->RealTime().nsec);
                    contact->set_collision1(tmp_col1);
                    contact->set_collision2(tmp_col2);
                    contact->set_allocated_time(time);
                    contact->set_world(world->Name());
                }
            }
        }
    }
}

/////////////////////////////////////////////////
void ContactWorldPlugin::changeSurface(ContactResponse & _msg)
{
    if (msg_req)
    {
        for (auto & request : msg_req->surface())
        {
            physics::ModelPtr model =
                world->ModelByName(request.model());
            if (!model) { return; }

            physics::LinkPtr link = model->GetLink(request.link());
            if (!link) { return; }

            physics::CollisionPtr col =
                link->GetCollision(request.collision());
            if (!col) { return; }

            physics::SurfaceParamsPtr surface = col->GetSurface();
            if (!surface) { return; }

            // Update surface properties
            surface->ProcessMsg(request.surface());

            gzdbg << "Changed surface properties" << std::endl;
        }
    }
}

}

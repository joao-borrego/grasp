/*!
    \file plugins/ContactSensorPlugin.cc
    \brief ContactSensor Gazebo plugin

    Plugin for handling contact sensors

    \author João Borrego : jsbruglie
*/

#include "ContactSensorPlugin.hh"

namespace gazebo {


/// \brief Class for private ContactSensor plugin data.
class ContactSensorPluginPrivate
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
GZ_REGISTER_SENSOR_PLUGIN(ContactSensorPlugin)

/////////////////////////////////////////////////
ContactSensorPlugin::ContactSensorPlugin() : SensorPlugin(),
    data_ptr(new ContactSensorPluginPrivate)
{
    gzmsg << "[ContactSensorPlugin] Launched plugin." << std::endl;
}

/////////////////////////////////////////////////
ContactSensorPlugin::~ContactSensorPlugin()
{
    gzmsg << "[ContactSensorPlugin] Unloaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void ContactSensorPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    this->sensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

    // Make sure the parent sensor is valid
    if (!this->sensor)
    {
        gzerr << "No contact sensor provided." << std::endl ;
        return;
    }

    // Connect to the sensor update event
    this->update_connection = this->sensor->ConnectUpdated(
      std::bind(&ContactSensorPlugin::onUpdate, this));
    // Make sure the parent sensor is active
    this->sensor->SetActive(true);

    // Setup transport node
    this->data_ptr->node = transport::NodePtr(new transport::Node());
    this->data_ptr->node->Init();
    // Subcribe to the monitored requests topic
    this->data_ptr->sub = this->data_ptr->node->Subscribe(REQUEST_TOPIC,
        &ContactSensorPlugin::onRequest, this);
    // Setup publisher for the response topic
    this->data_ptr->pub = this->data_ptr->node->
        Advertise<ContactSensorResponse>(RESPONSE_TOPIC);

    gzmsg << "[ContactSensorPlugin] Loaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void ContactSensorPlugin::onUpdate()
{
    // TODO
}

/////////////////////////////////////////////////
void ContactSensorPlugin::onRequest(ContactSensorRequestPtr &_msg)
{
    ContactSensorResponse msg;

    if (_msg->has_type()) {

        int type = _msg->type();
        if (type == REQ_IN_CONTACT)
        {
            if (_msg->has_in_contact()) {
                std::string name = _msg->in_contact();
                msg.set_type(RES_IN_CONTACT);
                if (this->inContact(name)) {
                    msg.set_in_contact(name);
                }
                this->data_ptr->pub->Publish(msg);
            }
        }
    }
}

/////////////////////////////////////////////////
bool ContactSensorPlugin::inContact(std::string & name)
{
    // Get all the contacts
    msgs::Contacts contacts;
    contacts = this->sensor->Contacts();
    // Check each of the contacts for a collision matching given name
    for (unsigned int i = 0; i < contacts.contact_size(); ++i)
    {
        std::string link_name = contacts.contact(i).collision2();
        if (link_name.find(name) != std::string::npos) {
            return true;
        }
    }
    return false;
}

}

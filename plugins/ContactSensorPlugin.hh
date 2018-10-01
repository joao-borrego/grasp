/*!
    \file plugins/ContactSensorPlugin.hh
    \brief ContactSensor Gazebo plugin

    Plugin for handling contact sensors

    \author João Borrego : jsbruglie
*/

#ifndef _CONTACT_SENSOR_PLUGIN_HH_
#define _CONTACT_SENSOR_PLUGIN_HH_

// Gazebo
#include "gazebo/common/Plugin.hh"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/sensors.hh>

// Custom messages
#include "contact_sensor_request.pb.h"
#include "contact_sensor_response.pb.h"

namespace ContactSensorPlugin {

    // Plugin messages

    /// Topic for incoming requests
    #define REQUEST_TOPIC   "~/grasp/contact"
     /// Topic for outgoing responses
    #define RESPONSE_TOPIC  "~/grasp/contact/response"

    /// Request for test contact
    #define REQ_IN_CONTACT   grasp::msgs::ContactSensorRequest::IN_CONTACT

    /// Response for test contact
    #define RES_IN_CONTACT   grasp::msgs::ContactSensorResponse::IN_CONTACT
}

namespace gazebo {

    /// Declaration for request message type
    typedef grasp::msgs::ContactSensorRequest ContactSensorRequest;
    /// Shared pointer declaration for request message type
    typedef const boost::shared_ptr<const grasp::msgs::ContactSensorRequest>
        ContactSensorRequestPtr;
    /// Declaration for response message type
    typedef grasp::msgs::ContactSensorResponse ContactSensorResponse;
    /// Shared pointer declaration for response message type
    typedef const boost::shared_ptr<const grasp::msgs::ContactSensorResponse>
        ContactSensorResponsePtr;

    // Forward declaration of private data class
    class ContactSensorPluginPrivate;

    /// \brief Contact sensor plugin
    class ContactSensorPlugin : public SensorPlugin
    {
        // Private attributes

        /// \brief Class with private attributes
        private: std::unique_ptr<ContactSensorPluginPrivate> data_ptr;
        /// \brief Sensor to which the plugin is attached
        private: sensors::ContactSensorPtr sensor;
        /// \brief Connection to world update event
        private: event::ConnectionPtr update_connection;

        // Public methods

        /// \brief Constructs the object
        public: ContactSensorPlugin();

        /// \brief Destroys the object
        public: virtual ~ContactSensorPlugin();

        /// \brief Loads the plugin
        /// \param _sensor The sensor pointer
        /// \param _sdf The sdf element pointer
        public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

        /// \brief Updates model on world update
        public: void onUpdate();

        /// \brief Callback function for handling incoming requests
        /// \param _msg  The message
        public: void onRequest(ContactSensorRequestPtr & _msg);

        /// \brief Returns if sensor is in contact with given collision model
        /// \param name Name of collision to test
        /// \return Whether sensor is in contact with collision model
        private: bool inContact(std::string & name);

    };
}

#endif

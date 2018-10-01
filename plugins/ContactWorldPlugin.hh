/*!
    \file plugins/ContactWorldPlugin.hh
    \brief Contact manager Gazebo world plugin

    \author João Borrego : jsbruglie
*/

#ifndef _CONTACT_WORLD_PLUGIN_HH_
#define _CONTACT_WORLD_PLUGIN_HH_

// Gazebo
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/common/Plugin.hh"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

// Custom messages
#include "contact_request.pb.h"
#include "contact_response.pb.h"

namespace ContactWorldPlugin {

    // Plugin messages

    /// Topic for incoming requests
    #define REQUEST_TOPIC   "~/grasp/contact"
     /// Topic for outgoing responses
    #define RESPONSE_TOPIC  "~/grasp/contact/response"
}

namespace gazebo {

    /// Declaration for request message type
    typedef grasp::msgs::ContactRequest ContactRequest;
    /// Shared pointer declaration for request message type
    typedef const boost::shared_ptr<const grasp::msgs::ContactRequest>
        ContactRequestPtr;
    /// Declaration for response message type
    typedef grasp::msgs::ContactResponse ContactResponse;
    /// Shared pointer declaration for response message type
    typedef const boost::shared_ptr<const grasp::msgs::ContactResponse>
        ContactResponsePtr;

    // Forward declaration of private data class
    class ContactWorldPluginPrivate;

    /// Contact manager world plugin
    class ContactWorldPlugin : public WorldPlugin
    {
        // Private attributes

        /// Class with private attributes
        private: std::unique_ptr<ContactWorldPluginPrivate> data_ptr;

        /// World pointer
        private: physics::WorldPtr world;
        /// Connection to world update event
        private: event::ConnectionPtr update_connection;

        /// Flag for message received
        private: bool recv_msg  {false};
        /// Flag for resetting contacts topic subscriber
        private: bool enabled   {false};
        /// Last received request message
        private: boost::shared_ptr<ContactRequest const> msg_req;

        // Public methods

        /// \brief Constructs the object
        public: ContactWorldPlugin();

        /// \brief Destroys the object
        public: virtual ~ContactWorldPlugin();

        /// \brief Loads the plugin
        /// \param _world The world pointer
        /// \param _sdf   The sdf element pointer
        public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

        /// \brief Callback on world update event
        /// Processes stored request, unsubscribes from contacts topic and clears messages.
        public: void onUpdate();

        /// \brief Callback on contact message received event
        /// Stores message for later processing in update callback
        /// \param _msg Received message
        public: void onContact(ConstContactsPtr & _msg);

        /// \brief Callback function for handling incoming requests
        /// Subscribes to "~/physics/contacts" topic, which decreases performance.
        /// Incoming contact messages are monitored in onContact callback function.
        /// \param _msg  The message
        public: void onRequest(ContactRequestPtr & _msg);

        /// \brief Callback function for handling outgoing responses
        /// \param _msg  The message
        public: void onResponse(ContactResponsePtr & _msg);

        /// \brief Check for collisions and update response message
        /// \param _msg Output message
        public: void checkCollision(ContactResponse & _msg);

        /// \brief Changes surface property from message
        /// \param _msg Input message
        /// \deprecated Redundant feature already implemented in external dependency
        public: void changeSurface(ContactResponse & _msg);
    };
}

#endif

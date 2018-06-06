/*!
    \file plugins/ContactWorldPlugin.hh
    \brief Contact manager Gazebo world plugin

    // TODO

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
    #define REQUEST_TOPIC   "~/grasp/contact/world"
     /// Topic for outgoing responses
    #define RESPONSE_TOPIC  "~/grasp/contact/world/response"
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

    // TODO
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
        /// Collisions to check
        private: std::string col1, col2;

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
        public: void onUpdate();

        /// TODO
        public: void onContact(ConstContactsPtr & _msg);

        /// \brief Callback function for handling incoming requests
        /// \param _msg  The message
        public: void onRequest(ContactRequestPtr & _msg);

        /// \brief Callback function for handling outgoing responses
        /// \param _msg  The message
        public: void onResponse(ContactResponsePtr & _msg);
    };
}

#endif

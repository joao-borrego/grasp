/*!
    \file utils/contacts_example.hh
    \brief TODO

    TODO

    \author João Borrego : jsbruglie
*/

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// I/O streams
#include <iostream>

// Custom messages
#include "contact_request.pb.h"
#include "contact_response.pb.h"

/// Command prompt
#define PROMPT "> "

/// Topic for incoming requests
#define REQUEST_TOPIC   "~/grasp/contact/"
 /// Topic for outgoing responses
#define RESPONSE_TOPIC  "~/grasp/contact/response"

/// Declaration for request message type
typedef grasp::msgs::ContactRequest ContactRequest;
/// Declaration for request aux message type
typedef grasp::msgs::Collision Collision;
/// Shared pointer declaration for response message type
typedef const boost::shared_ptr<const grasp::msgs::ContactResponse>
    ContactResponsePtr;

// Functions
void getContactBetween(gazebo::transport::PublisherPtr pub,
    const std::string & collision1,
    const std::string & collision2);

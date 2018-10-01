/*!
    \file examples/contacts_example.hh
    \brief Contact world plugin example

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
#define REQUEST_TOPIC   "~/grasp/contact"
 /// Topic for outgoing responses
#define RESPONSE_TOPIC  "~/grasp/contact/response"

/// Declaration for request aux message type
typedef grasp::msgs::CollisionRequest CollisionRequest;
/// Declaration for request aux message type
typedef grasp::msgs::SurfaceRequest SurfaceRequest;
/// Declaration for request message type
typedef grasp::msgs::ContactRequest ContactRequest;
/// Shared pointer declaration for response message type
typedef const boost::shared_ptr<const grasp::msgs::ContactResponse>
    ContactResponsePtr;

// Functions

/// \brief Obtain contact between two collision entities
/// \param pub Communication publisher
/// \param collision1 Name of first collision in pair to test
/// \param collision2 Name of second collision in pair to test
void getContactBetween(gazebo::transport::PublisherPtr pub,
    const std::string & collision1,
    const std::string & collision2);

/// \brief Change surface properties
/// \param pub Communication publisher
/// \param model Target model
/// \param link Target link
/// \param collision Target collision
void changeSurface(gazebo::transport::PublisherPtr pub,
    const std::string & model,
    const std::string & link,
    const std::string & collision);

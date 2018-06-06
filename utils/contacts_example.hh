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
#include "contact_sensor_request.pb.h"
#include "contact_sensor_response.pb.h"

/// Command prompt
#define PROMPT "> "

/// Topic for incoming requests
#define REQUEST_TOPIC   "~/grasp/contact/world"
 /// Topic for outgoing responses
#define RESPONSE_TOPIC  "~/grasp/contact/world/response"

/// Request for test contact
#define REQ_IN_CONTACT   grasp::msgs::ContactSensorRequest::IN_CONTACT
/// Response for test contact
#define RES_IN_CONTACT   grasp::msgs::ContactSensorResponse::IN_CONTACT

/// Declaration for request message type
typedef grasp::msgs::ContactSensorRequest ContactSensorRequest;
/// Shared pointer declaration for response message type
typedef const boost::shared_ptr<const grasp::msgs::ContactSensorResponse>
    ContactSensorResponsePtr;

// Functions
void getContactBetween(gazebo::transport::PublisherPtr pub,
    const std::string & collision1,
    const std::string & collision2);

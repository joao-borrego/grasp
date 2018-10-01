/*!
    \file examples/target_example.hh
    \brief Target object plugin example

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
#include "target_request.pb.h"
#include "target_response.pb.h"

/// Command prompt
#define PROMPT "> "

/// Topic monitored by target plugin for incoming requests
#define REQUEST_TOPIC   "~/grasp/target"
/// Topic to which target plugin publishes replies
#define RESPONSE_TOPIC  "~/grasp/target/response"

/// Get pose request
#define GET_POSE        grasp::msgs::TargetRequest::GET_POSE
/// Set pose request
#define SET_POSE        grasp::msgs::TargetRequest::SET_POSE
/// Get updated resting pose request
#define GET_REST_POSE   grasp::msgs::TargetRequest::GET_REST_POSE

/// Shared pointer declaration for response message type
typedef const boost::shared_ptr<const grasp::msgs::TargetResponse>
    TargetResponsePtr;

// Functions

/// \brief Sets target object pose
/// \param pub Publisher to target plugin topic
/// \param pose New target object pose
void setPose(gazebo::transport::PublisherPtr pub,
    ignition::math::Pose3d pose);

/// \brief Requests updated target object resting pose
/// \param pub Publisher to target plugin topic
void getRestingPose(gazebo::transport::PublisherPtr pub);

/// TODO
void onTargetResponse(TargetResponsePtr & _msg);

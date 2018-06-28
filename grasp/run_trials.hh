/*!
    \file grasp/run_trials.hh
    \brief Performs grasp trials

    TODO

    \author João Borrego : jsbruglie
*/

#ifndef _RUN_TRIALS_HH_
#define _RUN_TRIALS_HH_

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
// I/O streams
#include <iostream>
// Sleep
#include <chrono>
#include <thread>

// Custom messages
#include "MessageTypes.hh"
// Grasp representation
#include "Grasp.hh"
// Tools
#include "gen_grasps.hh"
#include "object_utils.hh"
// Debug streams
#include "debug.hh"

/// Height threshold
#define Z_LIFTED 0.2

#define SHADOWHAND

// Topics

/// Topic monitored by hand plugin for incoming requests
#define HAND_REQ_TOPIC      "~/hand"
/// Topic for hand plugin responses
#define HAND_RES_TOPIC      "~/hand/response"
/// Topic monitored by target plugin for incoming requests
#define TARGET_REQ_TOPIC    "~/grasp/target"
/// Topic for target plugin responses
#define TARGET_RES_TOPIC    "~/grasp/target/response"
/// Topic monitored by contacts plugin for incoming requests
#define CONTACT_REQ_TOPIC   "~/grasp/contact"
/// Topic for contacts plugin responses
#define CONTACT_RES_TOPIC   "~/grasp/contact/response"
/// Topic monitored by camera plugin for incoming requests
#define CAMERA_REQ_TOPIC    "~/grasp/rgbd"
/// Topic for camera plugin responses
#define CAMERA_RES_TOPIC    "~/grasp/rgbd/response"
/// Topic for Gazebo factory utility
#define FACTORY_TOPIC       "~/factory"
/// Topic for generic Gazebo requests
#define REQUEST_TOPIC       "~/request"

// Type enums

// Target Plugin

/// Get pose request
#define REQ_GET_POSE   grasp::msgs::TargetRequest::GET_POSE
/// Set pose request
#define REQ_SET_POSE   grasp::msgs::TargetRequest::SET_POSE
/// Update rest pose request
#define REQ_REST_POSE  grasp::msgs::TargetRequest::GET_REST_POSE
/// Current pose response
#define RES_POSE       grasp::msgs::TargetResponse::POSE
/// Updated rest pose response
#define RES_REST_POSE  grasp::msgs::TargetResponse::REST_POSE

// Camera plugin

/// Request to capture frame
#define REQ_CAPTURE    grasp::msgs::CameraRequest::CAPTURE

// Hand plugin

/// Position control
#define POSITION grasp::msgs::Target::POSITION
/// Velocity control
#define VELOCITY grasp::msgs::Target::VELOCITY

// Message type definitions

/// Declaration for hand message type
typedef grasp::msgs::Hand HandMsg;
/// Shared pointer declaration for hand message type
typedef const boost::shared_ptr<const grasp::msgs::Hand>
    HandMsgPtr;

/// Declaration for request message type
typedef grasp::msgs::TargetRequest TargetRequest;
/// Shared pointer declaration for request message type
typedef const boost::shared_ptr<const grasp::msgs::TargetRequest>
    TargetRequestPtr;
/// Declaration for response message type
typedef grasp::msgs::TargetResponse TargetResponse;
/// Shared pointer declaration for response message type
typedef const boost::shared_ptr<const grasp::msgs::TargetResponse>
    TargetResponsePtr;

/// Declaration for request aux message type
typedef grasp::msgs::CollisionRequest CollisionRequest;
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

/// Declaration for request message type
typedef grasp::msgs::CameraRequest CameraRequest;
/// Shared pointer declaration for request message type
typedef const boost::shared_ptr<const grasp::msgs::CameraRequest>
    CameraRequestPtr;
/// Declaration for response message type
typedef grasp::msgs::CameraResponse CameraResponse;
/// Shared pointer declaration for response message type
typedef const boost::shared_ptr<const grasp::msgs::CameraResponse>
    CameraResponsePtr;

// Functions

/// \brief Sets up gazebo communication pubs/subs
/// \param node Gazebo communication node pointer
/// \param pubs Resulting map of publishers
/// \param subs Resulting map of subscribers
void setupCommunications(
    gazebo::transport::NodePtr & node,
    std::map<std::string, gazebo::transport::PublisherPtr> & pubs,
    std::map<std::string, gazebo::transport::SubscriberPtr> & subs);

/// \brief Sets hand pose
/// \param pub Publisher to hand's topic
/// \param pose New hand pose
void setPose(gazebo::transport::PublisherPtr pub,
    ignition::math::Pose3d pose,
    double timeout=-1);

/// \brief Requests collisions in the world
/// \param pub Publisher to contact topic
/// \param target The target object name
/// \param hand The hand model name
void getContacts(gazebo::transport::PublisherPtr pub,
    std::string & target,
    std::string & hand);

/// TODO
void closeFingers(gazebo::transport::PublisherPtr pub,
    double timeout=-1);

/// TODO
void liftHand(gazebo::transport::PublisherPtr pub,
    double timeout=-1);

/// TODO
void getTargetPose(gazebo::transport::PublisherPtr pub, bool rest);

/// \brief Resets simulation world
/// \param pub Publisher to hand's topic
void reset(gazebo::transport::PublisherPtr pub);

/// \brief Attempts to grasp object
/// \param grasp The grasp configuration
/// \param pub   Publisher to hand's topic
/// \param pub   Publisher to grasp target topic
void tryGrasp(
    Grasp & grasp,
    std::map<std::string, gazebo::transport::PublisherPtr> & pubs,
    std::string & model_name);

/// TODO
void captureFrame(gazebo::transport::PublisherPtr pub);

/// \brief Returns whether to keep waiting for trigger
/// \param mutex   Mutex that protects trigger variable
/// \param trigger Trigger boolean variable
/// \return True as long as trigger is false, false otherwise
bool waitingTrigger(std::mutex & mutex, bool & trigger);

/// TODO
void onHandResponse(HandMsgPtr & _msg);

/// TODO
void onTargetResponse(TargetResponsePtr & _msg);

/// TODO
void onContactResponse(ContactResponsePtr & _msg);

/// TODO
void onCameraResponse(CameraResponsePtr & _msg);

/// TODO
void inline waitMs(int delay);

#endif

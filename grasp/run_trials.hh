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
#include "hand.pb.h"
#include "target_request.pb.h"
#include "target_response.pb.h"

// Grasp representation
#include "Grasp.hh"
// Tools
#include "gen_grasps.hh"
#include "object_utils.hh"

// Height threshold
#define Z_LIFTED 0.2

// Topics

/// Topic monitored by hand plugin for incoming requests
#define HAND_REQ_TOPIC      "~/hand"
/// Topic for hand plugin responses
#define HAND_RES_TOPIC    	"~/hand/response"
/// Topic monitored by target plugin for incoming requests
#define TARGET_REQ_TOPIC    "~/grasp/target"
/// Topic for target plugin responses
#define TARGET_RES_TOPIC    "~/grasp/target/response"
/// Topic for Gazebo factory utility
#define FACTORY_TOPIC       "~/factory"
/// Topic for generic Gazebo requests
#define REQUEST_TOPIC       "~/request"

// Type enums

/// Get pose request
#define REQ_GET_POSE   grasp::msgs::TargetRequest::GET_POSE
/// Set pose request
#define REQ_SET_POSE   grasp::msgs::TargetRequest::SET_POSE

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


// Functions

/// \brief Sets hand pose
/// \param pub Publisher to hand's topic
/// \param pose New hand pose
void setPose(gazebo::transport::PublisherPtr pub,
    ignition::math::Pose3d pose,
    double timeout=-1);

/// \brief Sets hand velocity
/// \param pub 		Publisher to hand's topic
/// \param pose 	New hand velocity vector
/// \param timeout	Timeout in seconds
void setVelocity(gazebo::transport::PublisherPtr pub,
    std::vector<double> & velocity,
    double timeout=-1);

/// \brief Sets hand's finger joint velocities
/// \param pub Publisher to hand's topic
/// \param pose New finger joint velocities vector
void setJointVelocities(gazebo::transport::PublisherPtr pub,
    std::vector<double> & velocities,
    double timeout=-1);

/// TODO
void getTargetPose(gazebo::transport::PublisherPtr pub);

/// \brief Resets simulation world
/// \param pub Publisher to hand's topic
void reset(gazebo::transport::PublisherPtr pub);

/// \brief Attempts to grasp object
/// \param grasp The grasp configuration
/// \param pub   Publisher to hand's topic
/// \param pub   Publisher to grasp target topic
void tryGrasp(
    Grasp & grasp,
    gazebo::transport::PublisherPtr pub_hand,
    gazebo::transport::PublisherPtr pub_target);

/// TODO
bool waitForTimeout();

/// TODO
bool waitForTrialEnd();

/// TODO
void onHandResponse(HandMsgPtr & _msg);

/// TODO
void onTargetResponse(TargetResponsePtr & _msg);

/// TODO
void inline waitMs(int delay);

#endif

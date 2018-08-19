/*!
    \file grasp/gen_rest_poses.hh
    \brief Obtain object rest poses 

    Drop each object in dataset from given pose and record rest pose

    \author João Borrego : jsbruglie
*/

#ifndef _GEN_REST_POSES_HH_
#define _GEN_REST_POSES_HH_

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
// Open YAML config files
#include "yaml-cpp/yaml.h"

// Custom messages
#include "MessageTypes.hh"
// Tools
#include "gen_grasps.hh"
#include "object_utils.hh"
// Debug streams
#include "debug.hh"
// Communication topics
#include "topics.hh"

// Type enums

// Target Plugin

/// Update rest pose request
#define REQ_REST_POSE  grasp::msgs::TargetRequest::GET_REST_POSE
/// Current pose response
#define RES_POSE       grasp::msgs::TargetResponse::POSE
/// Updated rest pose response
#define RES_REST_POSE  grasp::msgs::TargetResponse::REST_POSE

// Message type definitions

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

/// \brief Obtains usage string
/// \param argv_0 Name of the executable
/// \return String with command-line usage
const std::string getUsage(const char* argv_0);

/// \brief Parses command-line arguments
/// \param argc Argument count
/// \param argv Arguments
/// \param obj_cfg_dir Path to object dataset yaml
/// \param out_rest_dir Directory for output rest poses
void parseArgs(
    int argc,
    char** argv,
    std::string & obj_cfg_dir,
    std::string & out_rest_dir);

/// \brief Sets up gazebo communication pubs/subs
/// \param node Gazebo communication node pointer
/// \param pubs Resulting map of publishers
/// \param subs Resulting map of subscribers
void setupCommunications(
    gazebo::transport::NodePtr & node,
    std::map<std::string, gazebo::transport::PublisherPtr> & pubs,
    std::map<std::string, gazebo::transport::SubscriberPtr> & subs);

/// TODO
void obtainTargets(std::vector<std::string> & targets,
    const std::string & file_name);

/// \brief Requests target object rest pose
/// \param pub Publisher to target's topic
void getTargetRestPose(gazebo::transport::PublisherPtr pub);

/// \brief Returns whether to keep waiting for trigger
/// \param mutex   Mutex that protects trigger variable
/// \param trigger Trigger boolean variable
/// \return True as long as trigger is false, false otherwise
bool waitingTrigger(std::mutex & mutex, bool & trigger);

/// \brief Callback for Target plugin response
void onTargetResponse(TargetResponsePtr & _msg);

/// \brief Waits for `delay` milliseconds
/// \param delay Delay in milliseconds
void inline waitMs(int delay);

#endif

/*!
    \file grasp/gen_renders.hh
    \brief Obtain camera render per grasp candidate

    TODO

    \author João Borrego : jsbruglie
*/

#ifndef _GEN_RENDERS_HH_
#define _GEN_RENDERS_HH_

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
#include "MessageTopics.hh"
// Grasp representation
#include "Grasp.hh"
// Tools
#include "gen_grasps.hh"
#include "object_utils.hh"
// Debug streams
#include "debug.hh"

// Type enums

// Camera plugin

/// Request to capture frame
#define REQ_CAPTURE    grasp::msgs::CameraRequest::CAPTURE

// Message type definitions

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

/// Declaration for request message type
typedef grasp::msgs::TargetRequest TargetRequest;

// Functions

/// \brief Obtains usage string
/// \param argv_0 Name of the executable
/// \return String with command-line usage
const std::string getUsage(const char* argv_0);

/// \brief Parses command-line arguments
/// \param argc Argument count
/// \param argv Arguments
/// \param obj_cfg_dir Path to object dataset yaml
/// \param obj_rest_dir Path to object rest poses yaml
/// \param grasp_cfg_dir Path to grasp candidates yaml
/// \param out_img_dir Directory for output images
/// \param robot Robot to use
void parseArgs(
    int argc,
    char** argv,
    std::string & obj_cfg_dir,
    std::string & obj_rest_dir,
    std::string & grasp_cfg_dir,
    std::string & out_img_dir,
    std::string & robot);

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

/// TODO
void captureFrame(gazebo::transport::PublisherPtr pub);

/// \brief Returns whether to keep waiting for trigger
/// \param mutex   Mutex that protects trigger variable
/// \param trigger Trigger boolean variable
/// \return True as long as trigger is false, false otherwise
bool waitingTrigger(std::mutex & mutex, bool & trigger);

/// TODO
void onCameraResponse(CameraResponsePtr & _msg);

/// TODO
void inline waitMs(int delay);

#endif

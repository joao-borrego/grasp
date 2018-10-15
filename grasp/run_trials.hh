/*!
    \file grasp/run_trials.hh
    \brief Performs grasp trials

    Run full dynamic simulated grasp trials in Gazebo

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
// Threads
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
// Open YAML config files
#include "yaml-cpp/yaml.h"

// Custom messages
#include "MessageTypes.hh"
// Grasp representation
#include "Grasp.hh"
// Rest pose utils
#include "RestPose.hh"
// Interface for hand plugin
#include "Interface.hh"
// Tools
#include "object_utils.hh"
// Debug streams
#include "debug.hh"
// Randomiser class
#include "Randomiser.hh"

// GAP

// Custom messages
#include "dr_request.pb.h"
// Domain randomization plugin interface
#include "DRInterface.hh"

/// Config dictionary
typedef std::map<std::string,std::string> Config;

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
/// Reset request
#define REQ_RESET      grasp::msgs::TargetRequest::RESET
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

/// Invalid grasp flag
#define INVALID_GRASP -1.0

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

//
// Argument parsing and setup
//

/// \brief Obtains usage string
/// \param argv_0 Name of the executable
/// \return String with command-line usage
const std::string getUsage(const char* argv_0);

/// \brief Parses command-line arguments
/// \param argc Argument count
/// \param argv Arguments
/// \param config Configuration YAML node
void parseArgs(
    int argc,
    char** argv,
    Config & config);

/// \brief Sets up gazebo communication pubs/subs
/// \param node Gazebo communication node pointer
/// \param pubs Resulting map of publishers
/// \param subs Resulting map of subscribers
void setupCommunications(
    gazebo::transport::NodePtr & node,
    std::map<std::string, gazebo::transport::PublisherPtr> & pubs,
    std::map<std::string, gazebo::transport::SubscriberPtr> & subs);

//
// File I/O
//

/// \brief Obtain list of models' names in dataset yml
/// \param targets Output list of model names
/// \param file_name Input dataset config yml
void obtainTargets(std::vector<std::string> & targets,
    const std::string & file_name);

/// \brief Obtain list of grasps in yml files
/// \param grasp_cfg_dir Directory for grasp files
/// \param robot Target robot
/// \param object_name Target object name
/// \param grasps Output imported grasps
/// \returns Whether import was successful
bool importGrasps(const std::string & grasp_cfg_dir,
    const std::string & robot,
    const std::string & object_name,
    std::vector<Grasp> & grasps);

/// \brief Export set of metrics to file
/// \param trials_out_dir Output directory
/// \param robot Target robot name
/// \param object_name Target object name
/// \param grasps Set of grasps to export to file
void exportGraspMetrics(const std::string & trials_out_dir,
    const std::string & robot,
    const std::string & object_name,
    const std::vector<Grasp> & grasps);

//
// Gazebo plugin interaction
//

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
void checkHandCollisions(gazebo::transport::PublisherPtr pub,
    const std::string & hand,
    std::vector<std::string> & targets);

/// \brief Closes manipulator fingers
/// \param pub Publisher to hand topic
/// \param timeout Timeout in seconds
/// \warning Applies force directly
void closeFingers(gazebo::transport::PublisherPtr pub,
    double timeout=-1);

/// \brief Lifts robotic manipulator along positive z axis
/// \param pub Publisher to hand topic
/// \param timeout Timeout in seconds
void liftHand(gazebo::transport::PublisherPtr pub,
    double timeout=-1);

/// \brief Resets target object to rest pose
/// \param pub Publisher to target's topic
void resetTarget(gazebo::transport::PublisherPtr pub);

/// \brief Attempts to grasp object
/// \param grasp The grasp configuration
/// \param pubs  Map of publishers
/// \param model_name Target object model name
/// \return Grasp outcome metric
double tryGrasp(
    Grasp & grasp,
    Interface & interface,
    std::map<std::string, gazebo::transport::PublisherPtr> & pubs,
    const std::string & model_name);

// Synchronisation

/// \brief Waits for condition variable
/// \param timeout Timeout value in milliseconds
void waitForTrigger(int timeout=-1);

// Callback functions

/// TODO
void onHandResponse(HandMsgPtr & _msg);

/// TODO
void onTargetResponse(TargetResponsePtr & _msg);

/// TODO
void onContactResponse(ContactResponsePtr & _msg);

#endif

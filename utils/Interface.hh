/*!
    \file utils/Interface.hh
    \brief Grasp interface headers

    \author João Borrego : jsbruglie
*/

#ifndef _INTERFACE_HH_
#define _INTERFACE_HH_

#include <vector>
#include <map>
// Open YAML config files
#include "yaml-cpp/yaml.h"

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Custom utilities
#include "utils.hh"
// Debug streams
#include "debug.hh"
// Grasp shape representation
#include "GraspShape.hh"
// Custom messages
#include "target.pb.h"
#include "hand.pb.h"

// DEBUG
#include <iostream>

/// Topic monitored by hand plugin for incoming requests
#define HAND_PLUGIN_TOPIC "~/hand"

// Key for exiting program (Esc)
#define KEY_EXIT 27

/// \brief Hand interface
class Interface
{
    /// TODO
    public: static const grasp::msgs::Target_Type POSITION = 
        grasp::msgs::Target::POSITION;
    /// TODO
    public: static const grasp::msgs::Target_Type VELOCITY =
        grasp::msgs::Target::VELOCITY;

    /// TODO
    private: std::string robot_name;
    /// TODO
    private: gazebo::transport::NodePtr node;
    /// TODO
    private: gazebo::transport::PublisherPtr pub;
    /// TODO
    private: std::vector<std::string> joints;
    /// TODO
    private: std::map<std::string, double> state;
    /// TODO
    private: std::vector<GraspShape> grasps;

    /// \brief Constructor
    public: Interface();

    /// \brief Initalizes interface with config file
    /// \return True on success, false otherwise.
    public: bool init(
        const std::string & config_file,
        const std::string & robot);

    /// \brief Gets robot name
    /// \return Robot name string
    public: std::string getRobotName();

    /// \brief Sets hand pose
    /// \detailed Sets hand pose using virtual joints
    /// \param pose New hand pose
    /// \param timeout Timer value
    void setPose(ignition::math::Pose3d pose, double timeout=-1);

    /// \brief Sends reset signal to hand plugin.
    public: void reset(void);

    /// \brief Releases fingers and opens hand.
    /// \param timeout Timer value
    /// \param force Force joint position to target value
    void openFingers(double timeout=-1, bool force=false);

    /// \brief Clenches fingers and closes hand.
    /// \param timeout Timer value
    void closeFingers(double timeout=-1);

    /// \brief Raises hand.
    /// \param timeout Timer value
    void raiseHand(double timeout=-1);

    /// TODO
    public: void loop(void);

    /// TODO
    private: int processKeypress(char key);

    /// TODO
    private: void moveJoint(
        const char *joint, double value);

    /// TODO
    private: void moveFingers(double value);

    /// TODO
    private: void setJoints(
        std::vector<std::string> & joints,
        std::vector<double> & values);
};

#endif

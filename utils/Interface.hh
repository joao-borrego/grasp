/*!
    \file utils/Interface.hh
    \brief Grasp interface headers

    \author João Borrego : jsbruglie
*/

#ifndef _INTERFACE_HH_
#define _INTERFACE_HH_

#include <vector>
#include <map>

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "utils.hh"

// Custom messages
#include "target.pb.h"
#include "hand.pb.h"

// DEBUG
#include <iostream>

/// Position control
#define POSITION grasp::msgs::Target::POSITION
/// Velocity control
#define VELOCITY grasp::msgs::Target::VELOCITY

/// Topic monitored by hand plugin for incoming requests
#define HAND_PLUGIN_TOPIC "~/hand"

// Key for exiting program (Esc)
#define KEY_EXIT 27

/// \brief Hand interface
class Interface
{
    /// TODO
    private: gazebo::transport::NodePtr node;
    /// TODO
    private: gazebo::transport::PublisherPtr pub;
    /// TODO
    private: std::vector<std::string> joints;
    /// TODO
    private: std::vector<std::string> fingers;
    /// TODO
    private: double rest_finger_pos {0.0};
    /// TODO
    private: double bent_finger_pos {1.0}; 

    /// TODO
    private: std::map<std::string, double> state;


    /// \brief Constructor
    public: Interface();

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

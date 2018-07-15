/*!
    \file utils/hand_remote.hh
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

// Sleep
#include <chrono>
#include <thread>

// Custom messages
#include "target.pb.h"
#include "hand.pb.h"

// Interface
#include "Interface.hh"

/// Use shadowhand
#define SHADOWHAND

/// Command prompt
#define PROMPT "> "

/// Position control
#define POSITION grasp::msgs::Target::POSITION
/// Velocity control
#define VELOCITY grasp::msgs::Target::VELOCITY

/// Topic monitored by hand plugin for incoming requests
#define HAND_PLUGIN_TOPIC "~/hand"

// Functions

/// \brief Sets hand pose
/// \param pub Publisher to hand's topic
/// \param pose New hand pose
void setPose(gazebo::transport::PublisherPtr pub,
    ignition::math::Pose3d pose);

/// TODO
void moveFingers(gazebo::transport::PublisherPtr pub,
    bool close);

/// TODO
void liftHand(gazebo::transport::PublisherPtr pub);

/// \brief Resets simulation world
/// \param pub Publisher to hand's topic
void reset(gazebo::transport::PublisherPtr pub);

/// \brief Attempts to grasp object
/// \param pub Publisher to hand's topic
void tryGrasp(gazebo::transport::PublisherPtr pub);

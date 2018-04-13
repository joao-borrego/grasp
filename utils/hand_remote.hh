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
#include "hand.pb.h"

/// Command prompt
#define PROMPT "> "

/// Topic monitored by hand plugin for incoming requests
#define HAND_PLUGIN_TOPIC "~/hand"

// Functions

/// \brief Sets hand pose
/// \param pub Publisher to hand's topic
/// \param pose New hand pose
void setPose(gazebo::transport::PublisherPtr pub,
    ignition::math::Pose3d pose);

/// \brief Sets hand velocity
/// \param pub Publisher to hand's topic
/// \param pose New hand velocity vector
void setVelocity(gazebo::transport::PublisherPtr pub,
    std::vector<double> & velocity);

/// \brief Sets hand's finger joint velocities
/// \param pub Publisher to hand's topic
/// \param pose New finger joint velocities vector
void setJointVelocities(gazebo::transport::PublisherPtr pub,
    std::vector<double> & velocities);

/// \brief Resets simulation world
/// \param pub Publisher to hand's topic
void reset(gazebo::transport::PublisherPtr pub);

/// \brief Attempts to grasp object
/// \param pub Publisher to hand's topic
void tryGrasp(gazebo::transport::PublisherPtr pub);

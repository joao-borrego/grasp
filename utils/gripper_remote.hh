/*!
    \file utils/gripper_remote.hh
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
#include "gripper.pb.h"

/// Command prompt
#define PROMPT "> "

/// Topic monitored by Gripper plugin for incoming requests
#define GRIPPER_PLUGIN_TOPIC "~/gripper"

// Functions

/// \brief Sets gripper pose
/// \param pub Publisher to gripper's topic
/// \param pose New gripper pose
void setPose(gazebo::transport::PublisherPtr pub,
    ignition::math::Pose3d pose);

/// \brief Sets gripper velocity
/// \param pub Publisher to gripper's topic
/// \param pose New gripper velocity vector
void setVelocity(gazebo::transport::PublisherPtr pub,
    std::vector<double> & velocity);

/// \brief Opens gripper
/// \param pub Publisher to gripper's topic
void openGripper(gazebo::transport::PublisherPtr pub);

/// \brief Closes gripper
/// \param pub Publisher to gripper's topic
void closeGripper(gazebo::transport::PublisherPtr pub);

/// \brief Resets simulation world
/// \param pub Publisher to gripper's topic
void reset(gazebo::transport::PublisherPtr pub);

/// \brief Attempts to grasp object
/// \param pub Publisher to gripper's topic
void tryGrasp(gazebo::transport::PublisherPtr pub);

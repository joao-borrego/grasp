/*!
    \file grasp/object_utils.cc
    \brief Generate candidate grasps

    TODO

    \author João Borrego : jsbruglie
*/

#ifndef _OBJECT_UTILS_HH_
#define _OBJECT_UTILS_HH_

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// File streams
#include <fstream>

/// TODO
void spawnModelFromFile(
    gazebo::transport::PublisherPtr pub,
    std::string & file);

/// TODO
void removeModel(
    gazebo::transport::PublisherPtr pub,
    std::string & name);

#endif

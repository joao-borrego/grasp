/*!
    \file grasp/object_utils.hh
    \brief Object related utilities

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
// Open YAML config files
#include "yaml-cpp/yaml.h"

// Debug streams
#include "debug.hh"

/// \brief Spawns model from SDF description in file
/// \param pub Publisher to factory topic
/// \param file File with model SDF description
void spawnModelFromFile(
    gazebo::transport::PublisherPtr pub,
    const std::string & file);

/// \brief Spawns model from filename
/// \param pub Publisher to factory topic
/// \param pose Model initial pose
/// \param filename Name of model file
void spawnModelFromFilename(
    gazebo::transport::PublisherPtr pub,
    ignition::math::Pose3d & pose,
    const std::string & filename);

/// \brief Removes model
/// \param pub Publisher to requests topic
/// \param name Name of model to be removed
void removeModel(
    gazebo::transport::PublisherPtr pub,
    const std::string & name);

/// \brief Obtains object rest poses from file
/// \param file_name Input file name
/// \param targets List of objects in dataset
/// \param poses Output list of corresponding rest poses
void obtainRestPoses(
    const std::string & file_name,
    std::vector<std::string> & targets,
    std::vector<ignition::math::Pose3d> & poses);

#endif

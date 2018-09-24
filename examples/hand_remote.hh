/*!
    \file utils/hand_remote.hh
    \brief TODO

	TODO

    \author João Borrego : jsbruglie
*/

#ifndef _HAND_REMOTE_HH
#define _HAND_REMOTE_HH

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Interface
#include "Interface.hh"

// Functions

/// \brief Returns usage string.
/// \param argv_0 Executable name
/// \return Usage string.
const std::string getUsage(const char* argv_0);

/// \brief Parses command-line arguments
/// \param argc Argument count
/// \param argv Argument values
/// \param cfg_dir Robot configuration file
/// \param robot Robot name 
void parseArgs(
    int argc,
    char** argv,
    std::string & cfg_dir,
    std::string & robot);

#endif

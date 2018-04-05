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

// Custom messages
#include "gripper.pb.h"

/// Command prompt
#define PROMPT "> "

/// Topic monitored by Gripper plugin for incoming requests
#define GRIPPER_PLUGIN_TOPIC "~/gripper"

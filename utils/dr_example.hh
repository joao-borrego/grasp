/*!
    \file utils/target_example.hh
    \brief Domain randomization example client headers
    \author João Borrego : jsbruglie
*/

// Gazebo

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Custom messages
#include "dr_request.pb.h"
// Domain randomization plugin interface
#include "DRInterface.hh"

// Required fields workaround
#include <limits>

/// Declaration for request message type
typedef grasp::msgs::DRRequest DRRequest;

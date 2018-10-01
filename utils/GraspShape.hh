/*!
    \file utils/GraspShape.hh
    \brief Grasp shape representation

    \author João Borrego : jsbruglie
*/

#ifndef _GRASP_SHAPE_HH
#define _GRASP_SHAPE_HH

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Debug streams
#include "debug.hh"

// Functions

/// \brief Grasp pre/post shape class
class GraspShape
{
    /// Grasp shape name
    public: std::string name;
    /// Pre-grasp joint configurations
    public: std::vector < std::pair < std::string, double > > pre;
    /// Post-grasp joint configurations
    public: std::vector < std::pair < std::string, double > > post;

    // Functions

    /// \brief Constructs the object.
    /// \param _name The grasp configuration name
    GraspShape(std::string & _name);
};

#endif

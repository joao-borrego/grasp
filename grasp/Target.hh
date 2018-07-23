/*!
    \file grasp/Target.hh
    \brief Target object representation

    \author João Borrego : jsbruglie
*/

#ifndef _TARGET_HH_
#define _TARGET_HH_

// Gazebo
#include <gazebo/gazebo_client.hh>

/// \brief Grasp representation class
class Target
{
    // Public attributes

    /// Object name
    public: std::string name;
    /// Model resource path
    public: std::string path;

    /// \brief Constructor
    public: Target(
        std::string & name,
        std::string & path);
};

#endif

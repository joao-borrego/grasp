/*!
    \file grasp/Grasp.hh
    \brief Grasp representation

    \author João Borrego : jsbruglie
*/

#ifndef _GRASP_HH_
#define _GRASP_HH_

// Gazebo
#include <gazebo/gazebo_client.hh>
// Open YAML config files
#include "yaml-cpp/yaml.h"
// Debug streams
#include "debug.hh"

/// \brief Grasp representation class
class Grasp
{
    // Public attributes

    /// Grasp candidate name
    public: std::string name;
    /// Homogenous transform matrix from gripper to object reference frame
    public: ignition::math::Matrix4d t_gripper_object;
    /// Grasp outcome
    public: bool success {false};

    /// \brief Deafult constructor
    public: Grasp();
    /// \brief Constructor
    /// \param t_gripper_object Transform matrix from gripper to object frame
    public: Grasp(ignition::math::Matrix4d t_gripper_object_);

    /// \brief Load set of grasps from file
    /// \param file_name Input file name
    /// \param robot Target robot name
    /// \param grasps Set of grasps retrieved from file
    public: static void loadFromYml(
        const std::string & file_name,
        const std::string & robot,
        std::vector<Grasp> & grasps);
};

#endif

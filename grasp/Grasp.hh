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
    public: int id;
    /// Homogenous transform matrix from gripper to object reference frame
    public: ignition::math::Matrix4d t_gripper_object;
    /// Grasp success metric
    public: double metric {false};

    /// \brief Deafult constructor
    public: Grasp(int id_=0);

    /// \brief Constructor
    /// \param t_gripper_object_ Transform matrix from gripper to object frame
    /// \param id Grasp identifier
    public: Grasp(ignition::math::Matrix4d t_gripper_object_, int id_=0);

    /// \brief Load set of grasps from file
    /// \param file_name Input file name
    /// \param robot Target robot name
    /// \param object_name Target object name
    /// \param grasps Set of grasps retrieved from file
    public: static void loadFromYml(
        const std::string & file_name,
        const std::string & robot,
        const std::string & object_name,
        std::vector<Grasp> & grasps);

    /// \brief Export set of grasps to file
    /// \param file_name Output file name
    /// \param robot Target robot name
    /// \param object_name Target object name
    /// \param grasps Set of grasps to export to file
    public: static void writeToYml(
        const std::string & file_name,
        const std::string & robot,
        const std::string & object_name,
        const std::vector<Grasp> & grasps);
};

#endif

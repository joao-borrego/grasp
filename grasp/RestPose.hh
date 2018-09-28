/*!
    \file grasp/RestPose.hh
    \brief RestPose tool
    \author João Borrego : jsbruglie
*/

#ifndef _REST_POSE_HH_
#define _REST_POSE_HH_

// Gazebo
#include <gazebo/gazebo_client.hh>
// Open YAML config files
#include "yaml-cpp/yaml.h"
// Debug streams
#include "debug.hh"

/// \brief RestPose tools class
class RestPose
{
    /// \brief Load set of rest poses from yml file
    /// \param file_name Input file name
    /// \param object_name Target object name
    /// \param rest_poses Set of rest poses retrieved from file
    /// \param as_pose Import from 6D pose field
    /// \param as_matrix Import from 4x4 homogeneous transform matrix field
    public: static void loadFromYml(
        const std::string & file_name,
        const std::string & object_name,
        std::vector<ignition::math::Pose3d> & rest_poses,
        const bool as_pose=true,
        const bool as_matrix=false);

    /// \brief Writes set of rest poses to yml file
    /// \param file_name Output file name
    /// \param object_name Target object name
    /// \param rest_poses Set of rest poses to write from file
    /// \param as_pose Export as 6D pose
    /// \param as_matrix Export as first 3 rows of 4x4 homogeneous transform matrix
    public: static void writeToYml(
        const std::string & file_name,
        const std::string & object_name,
        const std::vector<ignition::math::Pose3d> & rest_poses,
        const bool use_pose=true,
        const bool as_matrix=false);
};

#endif

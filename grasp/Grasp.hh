/*!
    \file grasp/Grasp.hh
    \brief Grasp representation

    \author João Borrego : jsbruglie
*/

#ifndef _GRASP_HH_
#define _GRASP_HH_

// Gazebo
#include <gazebo/gazebo_client.hh>
// Eigen
#include <Eigen/Core>

/// \brief Grasp representation class
class Grasp
{
    // Public attributes

    /// Homogenous transform matrix to object reference frame
    public: Eigen::Matrix4d tf_matrix;
    /// Robot endpoint pose in world reference frame
    public: ignition::math::Pose3d pose;
    /// Grasp outcome
    public: bool success {false};

    /// \brief Constructor
    public: Grasp(Eigen::Matrix4d _tf_matrix);

};

#endif

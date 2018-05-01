/*!
    \file grasp/Grasp.hh
    \brief Grasp representation

    \author João Borrego : jsbruglie
*/

#ifndef _GRASP_HH_
#define _GRASP_HH_

// Gazebo
#include <gazebo/gazebo_client.hh>

/// \brief Grasp representation class
class Grasp
{
    // Public attributes

    /// Robot endpoint pose
    public: ignition::math::Pose3d pose;
    /// Grasp outcome
    public: bool success {false};

    /// \brief Constructor
    /// \param _pose Endpoint pose
    public: Grasp(
        const ignition::math::Pose3d & _pose
    );

};

#endif

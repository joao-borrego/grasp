/*!
    \file src/Grasp.cc
    \brief Grasp representation

    \author João Borrego : jsbruglie
*/

#include "Grasp.hh"

//////////////////////////////////////////////////
Grasp::Grasp(ignition::math::Matrix4d _tf_matrix):
    tf_matrix(_tf_matrix)
{
}

//////////////////////////////////////////////////
ignition::math::Pose3d Grasp::getPose(
    ignition::math::Pose3d ref_pose)
{
    return tf_matrix.Pose() + ref_pose;
}

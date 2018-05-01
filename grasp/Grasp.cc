/*!
    \file src/Grasp.cc
    \brief Grasp representation

    \author João Borrego : jsbruglie
*/

#include "Grasp.hh"

//////////////////////////////////////////////////
Grasp::Grasp(
    const ignition::math::Pose3d & _pose) :
        pose(_pose)
{
}

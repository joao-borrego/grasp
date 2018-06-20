/*!
    \file grasp/gen_grasps.cc
    \brief Generate candidate grasps

    TODO

    \author João Borrego : jsbruglie
*/

#ifndef _GEN_GRASPS_HH_
#define _GEN_GRASPS_HH_

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
// Eigen
#include <Eigen/Dense>
// Open YAML config files
#include "yaml-cpp/yaml.h"

// Grasp representation
#include "Grasp.hh"

/// TODO
void obtainGrasps(std::string file_name, std::vector<Grasp> & grasps);

/// \brief Get a random double in a given interval
///
/// Value is sampled from uniform distribution
///
/// \param min Interval lower bound
/// \param min Interval upper bound
/// \return Random double
double getRandomDouble(double min, double max);

#endif

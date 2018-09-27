/*!
    \file utils/Interface.hh
    \brief Grasp interface headers

    \author João Borrego : jsbruglie
*/

#ifndef _INTERFACE_HH_
#define _INTERFACE_HH_

#include <vector>
#include <map>
// Open YAML config files
#include "yaml-cpp/yaml.h"

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Custom utilities
#include "utils.hh"
// Debug streams
#include "debug.hh"
// Grasp shape representation
#include "GraspShape.hh"
// Custom messages
#include "target.pb.h"
#include "hand.pb.h"

// DEBUG
#include <iostream>

/// Topic monitored by hand plugin for incoming requests
#define HAND_PLUGIN_TOPIC "~/hand"

// Key for exiting program (Esc)
#define KEY_EXIT 27

/// \brief Hand interface
class Interface
{
    /// Direct force/torque application
    public: static const grasp::msgs::Target_Type FORCE =
        grasp::msgs::Target::FORCE;
    /// Position controller target
    public: static const grasp::msgs::Target_Type POSITION = 
        grasp::msgs::Target::POSITION;
    /// Velocity controller target
    public: static const grasp::msgs::Target_Type VELOCITY =
        grasp::msgs::Target::VELOCITY;

    /// Name of robot instance in Gazebo
    private: std::string robot_name;
    /// Gazebo communication node
    private: gazebo::transport::NodePtr node;
    /// Gazebo publisher
    private: gazebo::transport::PublisherPtr pub;
    /// Name of actuated robot joints
    private: std::vector<std::string> joints;
    /// Map with (name: value) states of each actuated joint
    private: std::map<std::string, double> state;
    /// Set of grasp shapes
    private: std::vector<GraspShape> grasps;
    /// Rigid transform from base link to gripper frame
    private: ignition::math::Matrix4d t_base_gripper;

    /// \brief Constructor
    public: Interface();

    /// \brief Initalizes interface with config file
    /// \return True on success, false otherwise.
    public: bool init(
        const std::string & config_file,
        const std::string & robot);

    // Getters

    /// \brief Gets robot name
    /// \return Robot name string
    public: std::string getRobotName();

    /// \brief Get T base to gripper frame
    /// \return Rigid transform from base link to gripper frame
    public: ignition::math::Matrix4d getTransformBaseGripper();

    // API

    /// \brief Sets hand pose
    /// \detailed Sets hand pose using virtual joints
    /// \param pose New hand pose
    /// \param timeout Timer value
    void setPose(ignition::math::Pose3d pose, double timeout=-1);

    /// \brief Sends reset signal to hand plugin.
    public: void reset(void);

    /// \brief Releases fingers and opens hand.
    /// \param timeout Timer value
    /// \param set_position Set joint position to target value
    void openFingers(double timeout=-1,
        bool set_position=false);

    /// \brief Clenches fingers and closes hand.
    /// \param timeout Timer value
    void closeFingers(double timeout=-1,
        bool apply_force=false);

    /// \brief Raises hand.
    /// \param timeout Timer value
    void raiseHand(double timeout=-1);

    /// TODO
    public: void loop(void);

    /// TODO
    private: int processKeypress(char key);

    /// TODO
    private: void moveJoint(
        const char *joint, double value);

    /// TODO
    private: void moveFingers(double value);

    /// TODO
    private: void setJoints(
        std::vector<std::string> & joints,
        std::vector<double> & values);
};

#endif

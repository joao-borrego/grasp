/*!
    \file grasp/Randomiser.hh
    \brief Randomiser

    \author João Borrego : jsbruglie
*/

#ifndef _RANDOMISER_HH_
#define _RANDOMISER_HH_

// Random distributions
#include <random>

// Gazebo
#include <gazebo/gazebo_client.hh>

// Custom messages
#include "dr_request.pb.h"
// Domain randomization plugin interface
#include "DRInterface.hh"

/// \brief Physics property with gaussian disturbance
class GaussianProperty
{
    /// \brief Property name
    public: std::string property; 
    /// \brief Gaussian mean
    public: double mean;
    /// \brief Gaussian standard deviation
    public: double std;
    /// \brief Scale factor
    public: double scale;
    /// \brief Gaussian distribution
    public: std::normal_distribution<double> dist;

    /// \brief Constructor
    /// \param property Property name
    /// \param mean Gaussian mean
    /// \param std Gaussian standard deviation
    /// \param scale Scaling factor
    public: GaussianProperty(
        const std::string & property,
        double mean,
        double std,
        double scale=0.0);
};

/// \brief Physics property with uniform disturbance
class UniformProperty
{
    /// \brief Property name
    public: std::string property; 
    /// \brief Uniform lower limit
    public: double a;
    /// \brief Uniform higher limit
    public: double b;
    /// \brief Scale factor
    public: double scale;
    /// \brief Uniform distribution
    public: std::uniform_real_distribution<double> dist;

    /// \brief Constructor
    /// \param property Property name
    /// \param mean Uniform mean
    /// \param std Uniform standard deviation
    /// \param scale Scaling factor
    public: UniformProperty(
        const std::string & property,
        double a,
        double b,
        double scale=0.0);
};

/// \brief Randomiser representation class
class Randomiser
{
    // Public attributes

    // Private attributes

    /// Mersenne Twister pseudorandom number generator
    private: std::mt19937 m_mt;

    /// Gaussian properties
    private: std::vector<GaussianProperty> n_p;
    /// Uniform properties
    private: std::vector<UniformProperty> u_p;

    /// \brief Constructor
    public: Randomiser(const std::string & config);
};

#endif

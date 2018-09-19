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

/// Declaration for request message type
typedef gap::msgs::DRRequest DRRequest;
/// Declaration for model command message type
typedef gap::msgs::ModelCmd ModelCmdMsg;

// Random samplers

/// \brief Abstract PRNG sampler class
class RandomSampler
{
    /// \brief Gets random sample
    /// \param gen Pseudo random number generator
    /// \returns Random sample
    public: virtual double sample(std::mt19937 & gen);
};

/// \brief Gaussian distribution sampler
class GaussianSampler : RandomSampler
{
    /// \brief Gaussian distribution
    public: std::normal_distribution<double> dist;

    /// \brief Gaussian mean
    private: double mean;
    /// \brief Gaussian standard deviation
    private: double std;

    /// \brief Constructor
    /// \param mean Gaussian mean
    /// \param std Gaussian standard deviation
    public: GaussianSampler(
        double mean,
        double std);

    /// \brief Gets random sample
    /// \param gen Pseudo random number generator
    /// \returns Random sample
    public: double sample(std::mt19937 & gen);
};

/// \brief Uniform/Log-uniform distribution sampler
class UniformSampler : RandomSampler
{
    /// \brief Uniform distribution
    public: std::uniform_real_distribution<double> dist;
 
    /// \brief Uniform lower limit
    private: double a;
    /// \brief Uniform higher limit
    private: double b;
    /// \brief Whether to use log-uniform instead
    private: bool log_uniform {false};

    /// \brief Constructor
    /// \param mean Uniform mean
    /// \param std Uniform standard deviation
    /// \param log_uniform Whether to use log-uniform or uniform dist
    public: UniformSampler(
        double a,
        double b,
        bool log_uniform=false);

    /// \brief Gets random sample
    /// \param gen Pseudo random number generator
    /// \returns Random sample
    public: double sample(std::mt19937 & gen);
};

// Random properties

/// \brief Abstract class for random property
class RandomProperty
{
    /// \brief Random distribution sampler
    private: RandomSampler sampler;
    /// \brief Whether term is additive or a scaling factor
    private: bool additive;
    /// \brief Model scale
    private: double scale; 

    /// \brief TODO
    public: virtual void fillMsg(DRRequest & msg);
};

/// \brief Model scale random property
class ModelScale : RandomProperty
{
    /// \brief Constructor
    /// TODO params
    public: ModelScale(
        RandomSampler & sampler_,
        bool additive_,
        double scale_,
        std::vector<std::string> & models_);

    /// \brief TODO
    public: void fillMsg(DRRequest & msg);
};

/// \brief Link mass random property
class LinkMass : RandomProperty
{
    /// \brief Constructor
    /// TODO params
    public: LinkMass(
        RandomSampler & sampler_,
        bool additive_,
        double scale_,
        std::vector<std::string> & links_,
        std::vector<double> & masses_);

    /// \brief TODO
    public: void fillMsg(DRRequest & msg);
};

/// \brief Friction coefficients random property
class FrictionCoefficient : RandomProperty
{
    /// \brief Constructor
    /// TODO params
    public: FrictionCoefficient(
        RandomSampler & sampler_,
        bool additive_,
        double scale_,
        std::vector<std::string> & links_,
        std::vector<double> & mu1_,
        std::vector<double> & mu2_,
        std::vector<double> & kp_,
        std::vector<double> & kd_);

    /// \brief TODO
    public: void fillMsg(DRRequest & msg);
};

/// \brief Joint damping coefficients random property
class JointDampingCoefficient : RandomProperty
{
    /// \brief Constructor
    /// TODO params
    public: JointDampingCoefficient(
        RandomSampler & sampler_,
        bool additive_,
        double scale_,
        std::vector<std::string> & joints_,
        std::vector<double> & damping_);

    /// \brief TODO
    public: void fillMsg(DRRequest & msg);
};

/// \brief P controller gains random property
class PGain : RandomProperty
{
    /// \brief Constructor
    /// TODO params
    public: PGain(
        RandomSampler & sampler_,
        bool additive_,
        double scale_,
        std::vector<std::string> & joints_,
        std::vector<double> & p_gains_);

    /// \brief TODO
    public: void fillMsg(DRRequest & msg);
};

/// \brief Joint limits random property
class JointLimit : RandomProperty
{
    /// \brief Constructor
    /// TODO params
    public: JointLimit(
        RandomSampler & sampler_,
        bool additive_,
        double scale_,
        std::vector<std::string> & joints_,
        std::vector<double> & lower_,
        std::vector<double> & upper_);

    /// \brief TODO
    public: void fillMsg(DRRequest & msg);
};

/// \brief Joint limits random property
class Gravity : RandomProperty
{
    /// \brief Constructor
    /// TODO params
    public: Gravity(
        RandomSampler & sampler_,
        bool additive_,
        double scale_,
        std::vector<double> gravity_);

    /// \brief TODO
    public: void fillMsg(DRRequest & msg);
};


// Randomiser class

/// \brief Randomiser representation class
class Randomiser
{
    // Private attributes

    /// Mersenne Twister pseudorandom number generator
    private: std::mt19937 m_mt;

    /// DRInterface API
    private: DRInterface api;

    /// \brief Constructor
    public: Randomiser(const std::string & config);
};

#endif

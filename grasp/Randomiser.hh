/*!
    \file grasp/Randomiser.hh
    \brief Randomiser

    \author João Borrego : jsbruglie
*/

#ifndef _RANDOMISER_HH_
#define _RANDOMISER_HH_

// Random distributions
#include <random>

// Open YAML config files
#include "yaml-cpp/yaml.h"

// Gazebo
#include <gazebo/gazebo_client.hh>

// Custom messages
#include "dr_request.pb.h"
// Domain randomization plugin interface
#include "DRInterface.hh"

// Debug streams
#include "debug.hh"

/// Declaration for request message type
typedef gap::msgs::DRRequest DRRequest;
/// Declaration for model command message type
typedef gap::msgs::ModelCmd ModelCmdMsg;

// Random samplers

/// \brief Abstract PRNG sampler class
class RandomSampler
{
    public: RandomSampler();

    /// \brief Gets random sample
    /// \param gen Pseudo random number generator
    /// \returns Random sample
    public: virtual double sample(std::mt19937 & gen);
};

/// \brief Gaussian distribution sampler
class GaussianSampler : public RandomSampler
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
class UniformSampler : public RandomSampler
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
    protected: RandomSampler sampler;
    /// \brief Whether term is additive or a scaling factor
    protected: bool additive;
    /// \brief Model scale
    protected: double scale;

    /// \brief TODO
    public: virtual void fillMsg(DRRequest & msg);
};

/// \brief Model scale random property
class ModelScale : public RandomProperty
{
    /// \brief Constructor
    /// TODO params
    public: ModelScale(
        RandomSampler & sampler_,
        bool additive_,
        bool scale_,
        std::vector<std::string> & models_);

    /// \brief TODO
    public: void fillMsg(DRRequest & msg);
};

/// \brief Link mass random property
class LinkMass : public RandomProperty
{
    /// \brief Constructor
    /// TODO params
    public: LinkMass(
        RandomSampler & sampler_,
        bool additive_,
        bool scale_,
        std::vector<std::string> & links_,
        std::vector<double> & masses_);

    /// \brief TODO
    public: void fillMsg(DRRequest & msg);
};

/// \brief Friction coefficients random property
class FrictionCoefficient : public RandomProperty
{
    /// \brief Constructor
    /// TODO params
    public: FrictionCoefficient(
        RandomSampler & sampler_,
        bool additive_,
        bool scale_,
        std::vector<std::string> & links_,
        std::vector<double> & mu1_,
        std::vector<double> & mu2_,
        std::vector<double> & kp_,
        std::vector<double> & kd_);

    /// \brief TODO
    public: void fillMsg(DRRequest & msg);
};

/// \brief Joint damping coefficients random property
class JointDampingCoefficient : public RandomProperty
{
    /// \brief Constructor
    /// TODO params
    public: JointDampingCoefficient(
        RandomSampler & sampler_,
        bool additive_,
        bool scale_,
        std::vector<std::string> & joints_,
        std::vector<double> & damping_);

    /// \brief TODO
    public: void fillMsg(DRRequest & msg);
};

/// \brief P controller gains random property
class PGain : public RandomProperty
{
    /// \brief Constructor
    /// TODO params
    public: PGain(
        RandomSampler & sampler_,
        bool additive_,
        bool scale_,
        std::vector<std::string> & joints_,
        std::vector<double> & p_gains_);

    /// \brief TODO
    public: void fillMsg(DRRequest & msg);
};

/// \brief Joint limits random property
class JointLimit : public RandomProperty
{
    /// \brief Constructor
    /// TODO params
    public: JointLimit(
        RandomSampler & sampler_,
        bool additive_,
        bool scale_,
        std::vector<std::string> & joints_,
        std::vector<double> & lower_,
        std::vector<double> & upper_);

    /// \brief TODO
    public: void fillMsg(DRRequest & msg);
};

/// \brief Joint limits random property
class Gravity : public RandomProperty
{
    /// \brief Constructor
    /// TODO params
    public: Gravity(
        RandomSampler & sampler_,
        bool additive_,
        bool scale_,
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

    /// \brief Randomised properties
    private: std::vector<RandomProperty> properties;

    // Public constants for yml config file
    // TODO - move to file?

    /// \brief Properties yml field string
    public: static const char CFG_PROPERTIES[];
    /// \brief Model scale yml field string
    public: static const char CFG_MODEL_SCALE[];
    /// \brief Link mass yml field string
    public: static const char CFG_LINK_MASS[];
    /// \brief Friction coefficients yml field string
    public: static const char CFG_FRICTION[];
    /// \brief Joint damping coefficients yml field string
    public: static const char CFG_JOINT_DAMPING[];
    /// \brief P controller gains yml field string
    public: static const char CFG_P_GAIN[];
    /// \brief Joint limits yml field string
    public: static const char CFG_JOINT_LIMIT[];
    /// \brief Gravity vector yml field string
    public: static const char CFG_GRAVITY[];
    /// \brief Distribution yml field string
    public: static const char CFG_DIST[];
    /// \brief Uniform distribution yml field string
    public: static const char CFG_UNIFORM[];
    /// \brief Log-uniform distribution yml field string
    public: static const char CFG_LOG_UNIFORM[];
    /// \brief Uniform distribution lower limit yml field string
    public: static const char CFG_UNIFORM_A[];
    /// \brief Uniform distribution upper limit yml field string
    public: static const char CFG_UNIFORM_B[];
    /// \brief Gaussian distribution yml field string
    public: static const char CFG_GAUSSIAN[];
    /// \brief Gaussian distribution mean yml field string
    public: static const char CFG_GAUSSIAN_MEAN[];
    /// \brief Gaussian distribution standard deviation yml field string
    public: static const char CFG_GAUSSIAN_STD[];
    /// \brief Model yml field string
    public: static const char CFG_MODEL[];
    /// \brief Links yml field string
    public: static const char CFG_LINKS[];
    /// \brief Joints yml field string
    public: static const char CFG_JOINTS[];
    /// \brief Vector yml field string
    public: static const char CFG_VECTOR[];
    /// \brief Target object yml keyword string
    public: static const char CFG_TARGET[];
    /// \brief Link mass yml field string
    public: static const char CFG_MASS[];
    /// \brief Joint damping yml field string
    public: static const char CFG_DAMPING[];
    /// \brief Joint lower limit yml field string
    public: static const char CFG_LOWER[];
    /// \brief Joint upper limit yml field string
    public: static const char CFG_UPPER[];
};

#endif

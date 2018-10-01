/*!
    \file grasp/Randomiser.hh
    \brief Randomiser class

    Helper class for streamlining physical property randomisation through
    <a href="https://github.com/jsbruglie/gap">GAP</a>.

    \author João Borrego : jsbruglie
*/

#ifndef _RANDOMISER_HH_
#define _RANDOMISER_HH_

// Random distributions
#include <random>
// Required fields workaround
#include <limits>

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
class IRandomSampler
{
    /// \brief Destructor
    public: virtual ~IRandomSampler() {};

    /// \brief Gets random sample
    /// \param gen Pseudo random number generator
    /// \returns Random sample
    public: virtual double sample(std::mt19937 & gen);
};

/// \brief Gaussian distribution sampler
class GaussianSampler : public IRandomSampler
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

    /// \brief Destructor
    public: ~GaussianSampler() override {};

    /// \brief Gets random sample
    /// \param gen Pseudo random number generator
    /// \returns Random sample
    public: double sample(std::mt19937 & gen) override;
};

/// \brief Uniform/Log-uniform distribution sampler
class UniformSampler : public IRandomSampler
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
    /// \param a Domain lower limit
    /// \param b Domain upper limit
    /// \param log_uniform Whether to use log-uniform or uniform dist
    public: UniformSampler(
        double a,
        double b,
        bool log_uniform=false);

    /// \brief Destructor
    public: ~UniformSampler() override {};

    /// \brief Gets random sample
    /// \param gen Pseudo random number generator
    /// \returns Random sample
    public: double sample(std::mt19937 & gen) override;
};

// Random properties

/// \brief Abstract class for random property
class RandomProperty
{
    /// \brief Random distribution sampler
    public: IRandomSampler *sampler;
    /// \brief Whether term is additive or a scaling factor
    public: bool additive;

    /// \brief Target object yml keyword string
    public: static const char CFG_TARGET_OBJ[];

    /// \brief Constructor
    /// \param sampler_ PRNG sampler
    /// \param additive_ Whether term is addictive or a scaling factor
    public: RandomProperty(
        IRandomSampler *sampler_,
        bool additive_);

    /// \brief Destructor
    public: virtual ~RandomProperty();

    /// \brief Fill DR request message
    /// \param msg DR request message
    /// \param api DR interface
    /// \param gen PRNG
    /// \param target Name of target object
    public: virtual void fillMsg(DRRequest & msg,
        DRInterface & api,
        std::mt19937 & gen,
        std::string & target);
};

/// \brief Model scale random property
class ModelScale : public RandomProperty
{
    /// \brief List of affected models
    public: std::vector<std::string> models;

    /// \brief Constructor
    /// \param sampler_ PRNG sampler
    /// \param additive_ Whether term is addictive or a scaling factor
    /// \param models_ List of affected models
    /// \warning std::vectors params are moved inside instance!
    public: ModelScale(
        IRandomSampler *sampler_,
        bool additive_,
        std::vector<std::string> & models_);

    /// \brief Destructor
    public: ~ModelScale() override {};

    /// \brief Fill DR request message
    /// \param msg DR request message
    /// \param api DR interface
    /// \param gen PRNG
    /// \param target Name of target object
    public: void fillMsg(DRRequest & msg,
        DRInterface & api,
        std::mt19937 & gen,
        std::string & target) override;
};

/// \brief Link mass random property
class LinkMass : public RandomProperty
{
    /// \brief List of affected models
    public: std::vector<std::string> models;
    /// \brief List of affected links
    public: std::vector<std::string> links;
    /// \brief Respective list of initial link masses
    public: std::vector<double> masses;

    /// \brief Constructor
    /// \param sampler_ PRNG sampler
    /// \param additive_ Whether term is addictive or a scaling factor
    /// \param models_ List of affected models
    /// \param links_ List of affected links
    /// \param masses_ Respective list of initial link masses
    /// \warning std::vectors params are moved inside instance!
    public: LinkMass(
        IRandomSampler *sampler_,
        bool additive_,
        std::vector<std::string> & models_,
        std::vector<std::string> & links_,
        std::vector<double> & masses_);

    /// \brief Destructor
    public: ~LinkMass() override {};

    /// \brief Fill DR request message
    /// \param msg DR request message
    /// \param api DR interface
    /// \param gen PRNG
    /// \param target Name of target object
    public: void fillMsg(DRRequest & msg,
        DRInterface & api,
        std::mt19937 & gen,
        std::string & target) override;
};

/// \brief Friction coefficients random property
class FrictionCoefficient : public RandomProperty
{
    /// \brief List of affected models
    public: std::vector<std::string> models;
    /// \brief List of affected links
    public: std::vector<std::string> links;
    /// \brief Respective list of initial mu1
    public: std::vector<double> mu1;
    /// \brief Respective list of initial mu2
    public: std::vector<double> mu2;
    /// \brief Respective list of initial kp
    public: std::vector<double> kp;
    /// \brief Respective list of initial kd
    public: std::vector<double> kd;

    /// \brief Constructor
    /// \param sampler_ PRNG sampler
    /// \param additive_ Whether term is addictive or a scaling factor
    /// \param models_ List of affected models
    /// \param links_ List of affected links
    /// \param mu1_ List of respective mu1
    /// \param mu2_ List of respective mu2
    /// \param kp_ List of respective kp
    /// \param kd_ List of respective kd
    /// \warning std::vectors params are moved inside instance!
    public: FrictionCoefficient(
        IRandomSampler *sampler_,
        bool additive_,
        std::vector<std::string> & models_,
        std::vector<std::string> & links_,
        std::vector<double> & mu1_,
        std::vector<double> & mu2_,
        std::vector<double> & kp_,
        std::vector<double> & kd_);

    /// \brief Destructor
    public: ~FrictionCoefficient() override {};

    /// \brief Fill DR request message
    /// \param msg DR request message
    /// \param api DR interface
    /// \param gen PRNG
    /// \param target Name of target object
    public: void fillMsg(DRRequest & msg,
        DRInterface & api,
        std::mt19937 & gen,
        std::string & target) override;
};

/// \brief Joint damping coefficients random property
class JointDampingCoefficient : public RandomProperty
{
    /// \brief List of affected models
    public: std::vector<std::string> models;
    /// \brief List of affected joints
    public: std::vector<std::string> joints;
    /// \brief Respective list of initial damping coefficients
    public: std::vector<double> damping;

    /// \brief Constructor
    /// \param sampler_ PRNG sampler
    /// \param additive_ Whether term is addictive or a scaling factor
    /// \param models_ List of affected models
    /// \param joints_ List of affected joints
    /// \param damping_ List of respective damping coefficients
    /// \warning std::vectors params are moved inside instance!
    public: JointDampingCoefficient(
        IRandomSampler *sampler_,
        bool additive_,
        std::vector<std::string> & models_,
        std::vector<std::string> & joints_,
        std::vector<double> & damping_);

    /// \brief Destructor
    public: ~JointDampingCoefficient() override {};

    /// \brief Fill DR request message
    /// \param msg DR request message
    /// \param api DR interface
    /// \param gen PRNG
    /// \param target Name of target object
    public: void fillMsg(DRRequest & msg,
        DRInterface & api,
        std::mt19937 & gen,
        std::string & target) override;
};

/// \brief P controller gains random property
class PGain : public RandomProperty
{
    /// \brief List of affected models
    public: std::vector<std::string> models;
    /// \brief List of affected joints
    public: std::vector<std::string> joints;
    /// \brief Respective list of controller types
    public: std::vector<int> types;
    /// \brief Respective list of initial P controller gains
    public: std::vector<double> p_gains;

    /// \brief Constructor
    /// \param sampler_ PRNG sampler
    /// \param additive_ Whether term is addictive or a scaling factor
    /// \param models_ List of affected models
    /// \param joints_ List of affected joints
    /// \param types_ List of respective controller types
    /// \param p_gains_ List of respective P controller gains
    /// \warning std::vectors params are moved inside instance!
    public: PGain(
        IRandomSampler *sampler_,
        bool additive_,
        std::vector<std::string> & models_,
        std::vector<std::string> & joints_,
        std::vector<int> & types_,
        std::vector<double> & p_gains_);

    /// \brief Destructor
    public: ~PGain() override {};

    /// \brief Fill DR request message
    /// \param msg DR request message
    /// \param api DR interface
    /// \param gen PRNG
    /// \param target Name of target object
    public: void fillMsg(DRRequest & msg,
        DRInterface & api,
        std::mt19937 & gen,
        std::string & target) override;
};

/// \brief Joint limits random property
class JointLimit : public RandomProperty
{
    /// \brief List of affected models
    public: std::vector<std::string> models;
    /// \brief List of affected joints
    public: std::vector<std::string> joints;
    /// \brief Respective list of initial joint lower limits
    public: std::vector<double> lower;
    /// \brief Respective list of initial joint upper limits
    public: std::vector<double> upper;

    /// \brief Constructor
    /// \param sampler_ PRNG sampler
    /// \param additive_ Whether term is addictive or a scaling factor
    /// \param models_ List of affected models
    /// \param joints_ List of affected joints
    /// \param lower_ List of respective joint lower limits
    /// \param upper_ List of respective joint upper limits
    /// \warning std::vectors params are moved inside instance!
    public: JointLimit(
        IRandomSampler *sampler_,
        bool additive_,
        std::vector<std::string> & models_,
        std::vector<std::string> & joints_,
        std::vector<double> & lower_,
        std::vector<double> & upper_);

    /// \brief Destructor
    public: ~JointLimit() override {};

    /// \brief Fill DR request message
    /// \param msg DR request message
    /// \param api DR interface
    /// \param gen PRNG
    /// \param target Name of target object
    public: void fillMsg(DRRequest & msg,
        DRInterface & api,
        std::mt19937 & gen,
        std::string & target) override;
};

/// \brief Joint limits random property
class Gravity : public RandomProperty
{
    /// \brief Initial gravity vector
    public: std::vector<double> gravity;

    /// \brief Constructor
    /// \param sampler_ PRNG sampler
    /// \param additive_ Whether term is addictive or a scaling factor
    /// \param gravity_ Initial gravity vector
    /// \warning std::vectors params are moved inside instance!
    public: Gravity(
        IRandomSampler *sampler_,
        bool additive_,
        std::vector<double> & gravity_);

    /// \brief Destructor
    public: ~Gravity() override {};

    /// \brief Fill DR request message
    /// \param msg DR request message
    /// \param api DR interface
    /// \param gen PRNG
    /// \param target Name of target object
    public: void fillMsg(DRRequest & msg,
        DRInterface & api,
        std::mt19937 & gen,
        std::string & target) override;
};


// Randomiser class

/// \brief Randomiser representation class
class Randomiser
{
    // Private attributes

    /// DRInterface API
    private: DRInterface api;
    /// \brief Randomised properties
    private: std::vector<RandomProperty*> properties;
    /// Mersenne Twister pseudorandom number generator
    private: std::mt19937 m_mt;
    /// Target object string
    private: std::string target {"TARGET_OBJECT"};

    /// \brief Constructor
    public: Randomiser(const std::string & config);

    /// \brief Destructor
    public: ~Randomiser();

    /// \brief Apply batch randomisation
    /// \param blocking Whether to wait for response
    public: void randomise(bool blocking=true);

    /// \brief Sets target object name
    /// \param target New target object name
    public: void setTargetName(const std::string & target);

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

    /// \brief Additive yml field string
    public: static const char CFG_ADDITIVE[];
    /// \brief Vector yml field string
    public: static const char CFG_VECTOR[];
    /// \brief Target yml keyword string
    public: static const char CFG_TARGET[];

    /// \brief Model yml field string
    public: static const char CFG_MODEL[];
    /// \brief Link yml field string
    public: static const char CFG_LINK[];
    /// \brief Joints yml field string
    public: static const char CFG_JOINT[];
    /// \brief Link mass yml field string
    public: static const char CFG_MASS[];
    /// \brief Joint damping yml field string
    public: static const char CFG_DAMPING[];
    /// \brief Joint lower limit yml field string
    public: static const char CFG_LOWER[];
    /// \brief Joint upper limit yml field string
    public: static const char CFG_UPPER[];
    /// \brief Surface mu1 yml field string
    public: static const char CFG_MU1[];
    /// \brief Surface mu2 yml field string
    public: static const char CFG_MU2[];
    /// \brief Surface kp yml field string
    public: static const char CFG_KP[];
    /// \brief Surface kd  yml field string
    public: static const char CFG_KD[];
    /// \brief Controller type yml field string
    public: static const char CFG_TYPE[];
    /// \brief P controller gain yml field string
    public: static const char CFG_P[];
    /// \brief Position controller keyword string
    public: static const char CFG_TYPE_POS[];
};

#endif

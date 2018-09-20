/*!
    \file grasp/Randomiser.cc
    \brief Randomiser

    \author João Borrego : jsbruglie
*/

#include "Randomiser.hh"

// Constants fro parsing yml config

const char Randomiser::CFG_PROPERTIES[]    = "properties";
const char Randomiser::CFG_MODEL_SCALE[]   = "model_scale";
const char Randomiser::CFG_LINK_MASS []    = "link_mass";
const char Randomiser::CFG_FRICTION[]      = "friction_coefficient";
const char Randomiser::CFG_JOINT_DAMPING[] = "joint_damping_coefficient";
const char Randomiser::CFG_P_GAIN[]        = "p_gain";
const char Randomiser::CFG_JOINT_LIMIT[]   = "joint_limit";
const char Randomiser::CFG_GRAVITY[]       = "gravity";

const char Randomiser::CFG_DIST[]          = "dist";
const char Randomiser::CFG_UNIFORM[]       = "uniform";
const char Randomiser::CFG_LOG_UNIFORM[]   = "loguniform";
const char Randomiser::CFG_UNIFORM_A[]     = "a";
const char Randomiser::CFG_UNIFORM_B[]     = "b";
const char Randomiser::CFG_GAUSSIAN[]      = "gaussian";
const char Randomiser::CFG_GAUSSIAN_MEAN[] = "mean";
const char Randomiser::CFG_GAUSSIAN_STD[]  = "std";

const char Randomiser::CFG_ADDITIVE[]      = "additive";
const char Randomiser::CFG_TARGET[]        = "target";
const char Randomiser::CFG_VECTOR[]        = "vector";

const char Randomiser::CFG_MODEL[]         = "model";
const char Randomiser::CFG_LINK[]          = "link";
const char Randomiser::CFG_JOINT[]         = "joint";
const char Randomiser::CFG_MASS[]          = "mass";
const char Randomiser::CFG_DAMPING[]       = "damping";
const char Randomiser::CFG_LOWER[]         = "lower";
const char Randomiser::CFG_UPPER[]         = "upper";
const char Randomiser::CFG_MU1[]           = "mu1";
const char Randomiser::CFG_MU2[]           = "mu2";
const char Randomiser::CFG_KP[]            = "kp";
const char Randomiser::CFG_KD[]            = "kd";
const char Randomiser::CFG_P[]             = "p";

const char Randomiser::CFG_TARGET_OBJ[]    = "TARGET_OBJECT";

//////////////////////////////////////////////////
Randomiser::Randomiser(const std::string & config):
    m_mt((std::random_device())())
{
    // Property type
    std::string p_type;
    // Distribution type
    std::string d_type;
    // Random sampler
    RandomSampler sampler;
    // Distribution parameters
    double a=0, b=0, mean=0, std=0;
    // Whether distribution is loguniform or uniform
    bool log_uniform = false;
    // Whether term is additive or scaling factor
    bool additive = false;
    // Number of targets
    int num_targets = 0;

    // Parse config file
    try
    {
        YAML::Node root = YAML::LoadFile(config);
        YAML::Node properties = root[CFG_PROPERTIES];
        for (const auto & property : properties)
        {
            p_type = property.first.as<std::string>();
            // TODO - Find decent solution for this?
            for (const auto & params : root[CFG_PROPERTIES][p_type][CFG_DIST])
            {
                d_type = params.first.as<std::string>();
            }

            // Handle different distributions
            if (d_type == CFG_GAUSSIAN)
            {
                mean = root[CFG_PROPERTIES][p_type][CFG_DIST]
                    [d_type][CFG_GAUSSIAN_MEAN].as<double>();
                std = root[CFG_PROPERTIES][p_type][CFG_DIST]
                    [d_type][CFG_GAUSSIAN_MEAN].as<double>();
                sampler = GaussianSampler(mean, std);
            }
            else if (d_type == CFG_UNIFORM || d_type == CFG_LOG_UNIFORM)
            {
                a = root[CFG_PROPERTIES][p_type][CFG_DIST]
                    [d_type][CFG_UNIFORM_A].as<double>();
                b = root[CFG_PROPERTIES][p_type][CFG_DIST]
                    [d_type][CFG_UNIFORM_B].as<double>();
                log_uniform = (d_type == CFG_LOG_UNIFORM);
                sampler = UniformSampler(a, b, log_uniform);
            }
            else
            {
                // TODO - Error handling
                continue;
            }
            additive = root[CFG_PROPERTIES][p_type][CFG_ADDITIVE].as<bool>();

            // Handle different properties
            if (p_type == CFG_MODEL_SCALE)
            {
                std::vector<std::string> models;
                num_targets = root[CFG_PROPERTIES][p_type][CFG_TARGET].size();
                for (int i = 0; i < num_targets; i++)
                {
                    YAML::Node target = root[CFG_PROPERTIES][p_type][CFG_TARGET][i];
                    models.push_back(target[CFG_MODEL].as<std::string>());
                }
                ModelScale property(sampler, additive, models);
            }
            else if (p_type == CFG_LINK_MASS)
            {
                std::vector<std::string> models;
                std::vector<std::string> links;
                std::vector<double> masses;
                num_targets = root[CFG_PROPERTIES][p_type][CFG_TARGET].size();
                for (int i = 0; i < num_targets; i++)
                {
                    YAML::Node target = root[CFG_PROPERTIES][p_type][CFG_TARGET][i];
                    models.push_back(target[CFG_MODEL].as<std::string>());
                    links.push_back(target[CFG_LINK].as<std::string>());
                    masses.push_back(target[CFG_MASS].as<double>());
                }
                LinkMass property(sampler, additive, models, links, masses);
            }
            else if (p_type == CFG_FRICTION)
            {
                std::vector<std::string> models;
                std::vector<std::string> links;
                std::vector<double> mu1;
                std::vector<double> mu2;
                std::vector<double> kp;
                std::vector<double> kd;
                num_targets = root[CFG_PROPERTIES][p_type][CFG_TARGET].size();
                for (int i = 0; i < num_targets; i++)
                {
                    YAML::Node target = root[CFG_PROPERTIES][p_type][CFG_TARGET][i];
                    models.push_back(target[CFG_MODEL].as<std::string>());
                    links.push_back(target[CFG_LINK].as<std::string>());
                    mu1.push_back(target[CFG_MU1].as<double>());
                    mu2.push_back(target[CFG_MU2].as<double>());
                    kp.push_back(target[CFG_KP].as<double>());
                    kd.push_back(target[CFG_KD].as<double>());
                }
                FrictionCoefficient property(sampler, additive, models, links,
                    mu1, mu2, kp, kd);
            }
            else if (p_type == CFG_JOINT_DAMPING)
            {
                std::vector<std::string> models;
                std::vector<std::string> joints;
                std::vector<double> damping;
                num_targets = root[CFG_PROPERTIES][p_type][CFG_TARGET].size();
                for (int i = 0; i < num_targets; i++)
                {
                    YAML::Node target = root[CFG_PROPERTIES][p_type][CFG_TARGET][i];
                    models.push_back(target[CFG_MODEL].as<std::string>());
                    joints.push_back(target[CFG_JOINT].as<std::string>());
                    damping.push_back(target[CFG_DAMPING].as<double>());
                }
                JointDampingCoefficient property(sampler, additive, models, joints,
                    damping);
            }
            else if (p_type == CFG_P_GAIN)
            {
                std::vector<std::string> models;
                std::vector<std::string> joints;
                std::vector<double> p_gains;
                num_targets = root[CFG_PROPERTIES][p_type][CFG_TARGET].size();
                for (int i = 0; i < num_targets; i++)
                {
                    YAML::Node target = root[CFG_PROPERTIES][p_type][CFG_TARGET][i];
                    models.push_back(target[CFG_MODEL].as<std::string>());
                    joints.push_back(target[CFG_JOINT].as<std::string>());
                    p_gains.push_back(target[CFG_P].as<double>());
                }
                PGain property(sampler, additive, models, joints,
                    p_gains);
            }
            else if (p_type == CFG_JOINT_LIMIT)
            {
                std::vector<std::string> models;
                std::vector<std::string> joints;
                std::vector<double> lower;
                std::vector<double> upper;
                num_targets = root[CFG_PROPERTIES][p_type][CFG_TARGET].size();
                for (int i = 0; i < num_targets; i++)
                {
                    YAML::Node target = root[CFG_PROPERTIES][p_type][CFG_TARGET][i];
                    models.push_back(target[CFG_MODEL].as<std::string>());
                    joints.push_back(target[CFG_JOINT].as<std::string>());
                    lower.push_back(target[CFG_LOWER].as<double>());
                    upper.push_back(target[CFG_UPPER].as<double>());
                }
                JointLimit property(sampler, additive, models, joints,
                    lower, upper);
            }
            else if (p_type == CFG_GRAVITY)
            {
                std::vector<double> gravity;
                YAML::Node target = root[CFG_PROPERTIES][p_type][CFG_VECTOR];
                gravity = target.as<std::vector<double>>();
                Gravity property(sampler, additive, gravity);
            }
        }
    }
    catch (YAML::Exception& yamlException)
    {
        std::cerr << "[Randomiser] Unable to parse " << config << "\n";
    }

    debugPrintTrace("Parsed " << config << " successfully.");
}

// Random properties

//////////////////////////////////////////////////
RandomProperty::RandomProperty(
    RandomSampler & sampler_,
    bool additive_) :
        sampler(sampler_), additive(additive_)
{
}

//////////////////////////////////////////////////
void RandomProperty::fillMsg(DRRequest & msg)
{
}

//////////////////////////////////////////////////
ModelScale::ModelScale(
    RandomSampler & sampler_,
    bool additive_,
    std::vector<std::string> & models_):
        RandomProperty(sampler_, additive_),
        models(std::move(models_))
{
}

//////////////////////////////////////////////////
void ModelScale::fillMsg(DRRequest & msg)
{
}

//////////////////////////////////////////////////
LinkMass::LinkMass(
    RandomSampler & sampler_,
    bool additive_,
    std::vector<std::string> & models_,
    std::vector<std::string> & links_,
    std::vector<double> & masses_):
        RandomProperty(sampler_, additive_),
        models(std::move(models_)), links(std::move(links_)),
        masses(std::move(masses_))
{
}

//////////////////////////////////////////////////
FrictionCoefficient::FrictionCoefficient(
    RandomSampler & sampler_,
    bool additive_,
    std::vector<std::string> & models_,
    std::vector<std::string> & links_,
    std::vector<double> & mu1_,
    std::vector<double> & mu2_,
    std::vector<double> & kp_,
    std::vector<double> & kd_):
        RandomProperty(sampler_, additive_),
        models(std::move(models_)), links(std::move(links_)),
        mu1(std::move(mu1_)), mu2(std::move(mu2_)),
        kp(std::move(kp_)), kd(std::move(kd_))
{
}

//////////////////////////////////////////////////
JointDampingCoefficient::JointDampingCoefficient(
    RandomSampler & sampler_,
    bool additive_,
    std::vector<std::string> & models_,
    std::vector<std::string> & joints_,
    std::vector<double> & damping_):
        RandomProperty(sampler_, additive_),
        models(std::move(models_)), joints(std::move(joints_)),
        damping(std::move(damping_))
{
}

//////////////////////////////////////////////////
PGain::PGain(
    RandomSampler & sampler_,
    bool additive_,
    std::vector<std::string> & models_,
    std::vector<std::string> & joints_,
    std::vector<double> & p_gains_):
        RandomProperty(sampler_, additive_),
        models(std::move(models_)), joints(std::move(joints_)),
        p_gains(std::move(p_gains_))
{
}

//////////////////////////////////////////////////
JointLimit::JointLimit(
    RandomSampler & sampler_,
    bool additive_,
    std::vector<std::string> & models_,
    std::vector<std::string> & joints_,
    std::vector<double> & lower_,
    std::vector<double> & upper_):
        RandomProperty(sampler_, additive_),
        models(std::move(models_)), joints(std::move(joints_)),
        lower(std::move(lower_)), upper(std::move(upper_))
{
}

//////////////////////////////////////////////////
Gravity::Gravity(
    RandomSampler & sampler_,
    bool additive_,
    std::vector<double> & gravity_):
        RandomProperty(sampler_, additive_),
        gravity(std::move(gravity_))
{
}

// Random samplers

//////////////////////////////////////////////////
double RandomSampler::sample(std::mt19937 & gen)
{
    return 0.0;
}

//////////////////////////////////////////////////
GaussianSampler::GaussianSampler(double mean_, double std_):
    mean(mean_), std(std_), dist(mean_, std_)
{
}

//////////////////////////////////////////////////
double GaussianSampler::sample(std::mt19937 & gen)
{
    return dist(gen);
}

//////////////////////////////////////////////////
UniformSampler::UniformSampler(double a_, double b_, bool log_uniform_):
    a(a_), b(b_), dist(a_, b_), log_uniform(log_uniform_)
{
}

//////////////////////////////////////////////////
double UniformSampler::sample(std::mt19937 & gen)
{
    if (log_uniform) {
        return std::exp(dist(gen));
    } else {
        return dist(gen);
    }
}

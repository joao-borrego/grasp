/*!
    \file grasp/Randomiser.cc
    \brief Randomiser

    \author João Borrego : jsbruglie
*/

#include "Randomiser.hh"

//////////////////////////////////////////////////
Randomiser::Randomiser(const std::string & config):
    m_mt((std::random_device())())
{

}

//////////////////////////////////////////////////
GaussianProperty::GaussianProperty(
    const std::string & property_, double mean_,
    double std_, double scale_):
        property(property_), mean(mean_), std(std_), scale(scale_), dist(mean_, std_)
{
}

//////////////////////////////////////////////////
UniformProperty::UniformProperty(
    const std::string & property_, double a_,
    double b_, double scale_):
        property(property_), a(a_), b(b_), scale(scale_), dist(a_, b_)
{
}

/*!
    \file utils/utils.hh
    \brief Useful 3rd party  functions

    \author Vinay Sajip : vsajip
*/

#ifndef _UTILS_HH_
#define _UTILS_HH_

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <stdlib.h>
// Random
#include <random>

/// \brief Checks for key press
///
/// \details As seen in
/// <a href="https://gist.github.com/vsajip/1864660">
///   this gist
/// </a>
///
/// \return True on key press, False otherwise
int kbhit();

/// \brief Get a random integer in a given interval
///
/// Value is sampled from uniform distribution
///
/// \param min Interval lower bound
/// \param max Interval upper bound
/// \return Random integer
int getRandomInt(int min, int max);

/// \brief Get a random double in a given interval
///
/// Value is sampled from uniform distribution
///
/// \param min Interval lower bound
/// \param max Interval upper bound
/// \return Random double
double getRandomDouble(double min, double max);

#endif

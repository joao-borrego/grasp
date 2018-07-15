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

/// \brief Checks for key press
///
/// \detailed As seen in
/// <a href="https://gist.github.com/vsajip/1864660">
///   this gist
/// </a>
///
/// \return True on key press, False otherwise
int kbhit();

#endif

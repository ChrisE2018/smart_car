/*
 * math.cpp
 *
 *  Created on: Jan 1, 2023
 *      Author: cre
 */

#include "robot_math.hpp"

bool is_overflow (const double x)
{
    // constants determined empirically
    return (x > 4294967040.0 || x < -4294967040.0);
}

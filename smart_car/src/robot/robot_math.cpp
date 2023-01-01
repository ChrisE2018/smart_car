/*
 * math.cpp
 *
 *  Created on: Jan 1, 2023
 *      Author: cre
 */

#include "robot_math.hpp"
#include "math.h"

bool is_overflow (const double x)
{
    // constants determined empirically
    return (x > 4294967040.0 || x < -4294967040.0);
}

float angle_delta (const float desired, const float actual)
{
    float result = desired - actual;
    if (result > M_PI)
    {
        result -= 2.0 * M_PI;
    }
    else if (result < -M_PI)
    {
        result += 2.0 * M_PI;
    }
    return result;
}

/*
 * math.cpp
 *
 *  Created on: Jan 1, 2023
 *      Author: cre
 */

#include "robot_math.hpp"
#include "math.h"

const float PI_2 = M_PI + M_PI;

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
        result -= PI_2;
    }
    else if (result < -M_PI)
    {
        result += PI_2;
    }
    return result;
}

float normalize_angle (const float angle)
{
    float result = angle;
    while (result > PI_2)
    {
        result -= PI_2;
    }
    while (result < -PI_2)
    {
        result += PI_2;
    }
    return result;
}

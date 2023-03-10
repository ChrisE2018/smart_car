/*
 * Copyright (c) 2023 by Christopher Eliot.
 *
 * robot_math.hpp
 *
 *  Created on: Jan 1, 2023
 *      Author: cre
 */

#pragma once

bool is_overflow (const double x);

float angle_delta (const float desired, const float actual);
float normalize_angle (const float angle);


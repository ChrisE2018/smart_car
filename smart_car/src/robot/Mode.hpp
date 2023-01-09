/*
 * Enum.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#pragma once

#include <ostream>

enum Mode
{
    COMMAND_MODE = 0,
    DEMO_MODE,
    GOAL_MODE,
    WALL_MODE
};

std::ostream& operator<< (std::ostream &lhs, Mode mode);

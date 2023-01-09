/*
 * Mode.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "Mode.hpp"

std::ostream& operator<< (std::ostream &lhs, const Mode mode)
{
    switch (mode)
    {
        case Mode::COMMAND_MODE:
            lhs << "COMMAND_MODE";
            break;
        case Mode::DEMO_MODE:
            lhs << "DEMO_MODE";
            break;
        case Mode::GOAL_MODE:
            lhs << "GOAL_MODE";
            break;
        case Mode::WALL_MODE:
            lhs << "WALL_MODE";
            break;
    }
    return lhs;
}

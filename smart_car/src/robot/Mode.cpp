/*
 * Mode.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "Mode.hpp"

std::ostream& operator<< (std::ostream &lhs, Mode mode)
{
    switch (mode)
    {
        case COMMAND_MODE:
            lhs << "COMMAND_MODE";
            break;
        case DEMO_MODE:
            lhs << "DEMO_MODE";
            break;
        case GOAL_MODE:
            lhs << "GOAL_MODE";
            break;
        case WALL_MODE:
            lhs << "WALL_MODE";
            break;
    }
    return lhs;
}

/*
 * Level.hpp
 *
 *  Created on: Jan 20, 2023
 *      Author: cre
 */

#pragma once

enum class Level
{
    error, warning, info, debug, data, none
};

const char* stringify (const Level level);


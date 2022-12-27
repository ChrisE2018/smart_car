/*
 * Executor.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include "Arduino.h"
#include <vector>

class Executor
{
    public:
        Executor ();
        virtual ~Executor () = default;

        virtual void execute_command (const std::vector<String> words);
};


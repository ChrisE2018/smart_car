/*
 * Executor.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include "Arduino.h"

class Executor
{
    public:
        Executor ();
        virtual ~Executor () = default;

        virtual void execute_command (const int word_counts, const String words[]);
};


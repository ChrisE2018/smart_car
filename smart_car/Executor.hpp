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

        void execute_command (const int word_counts, String words[]);
};


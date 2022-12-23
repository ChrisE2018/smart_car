/*
 * Parser.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include "Arduino.h"
#include "Executor.hpp"

class Parser
{
    public:
        Parser (Executor& executor);
        void handle_command ();
        int get_words (const String command, String result[], int max_words);

    private:
        Executor& executor;
};


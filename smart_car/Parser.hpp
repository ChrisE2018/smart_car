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
        Parser ();
        void handle_command (Executor &executor);

    private:
        int get_words (const String command, String result[], int max_words);
};


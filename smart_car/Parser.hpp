/*
 * Parser.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include "Arduino.h"
#include "Executor.hpp"
#include <vector>

class Parser
{
    public:
        Parser (HardwareSerial& serial);
        void handle_command (Executor &executor);

    private:
        HardwareSerial& serial;
        void get_words (const String command, std::vector<String>& words);
};


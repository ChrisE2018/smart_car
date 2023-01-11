/*
 * Parser.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include <Arduino.h>
#include <vector>
#include "Executor.hpp"

class Parser
{
    public:
        Parser (HardwareSerial &serial);
        void handle_command (Executor &executor);

    private:
        const int buffer_size = 80;

        HardwareSerial &serial;
        void get_words (const String& command, std::vector<String>& words);
};


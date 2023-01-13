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
        bool has_input();
        bool handle_command (Executor &executor);

    private:
        static constexpr int buffer_size = 80;

        HardwareSerial &serial;
        void get_words (const String& command, std::vector<String>& words);
};


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
        Parser (HardwareSerial& serial);
        void handle_command (Executor &executor);

    private:
        HardwareSerial& serial;
        int get_words (const String command, String result[], int max_words);
};


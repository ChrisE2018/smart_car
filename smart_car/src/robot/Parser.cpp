/*
 * Copyright (c) 2023 by Christopher Eliot.
 *
 * Parser.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Parser.hpp"

Parser::Parser (HardwareSerial &serial) :
                serial(serial)
{
}

bool Parser::has_input ()
{
    return serial.available();
}

bool Parser::handle_command (Executor &executor)
{
    if (serial.available())
    {
        char buffer[buffer_size];
        int i = 0;
        while (i < buffer_size)
        {
            if (serial.available())
            {
                const char cmd = serial.read();
                if (cmd == '\n' || cmd == '\r')
                {
                    if (i > 0)
                    {
                        buffer[i] = '\0';
                        const String command(buffer);
                        std::vector<String> words;
                        //serial.print(F("execute: "));
                        //serial.println(command);
                        get_words(command, words);
                        if (!words.empty())
                        {
                            executor.execute_command(words);
                            Serial.println(F("[executed]"));
                            Serial.println();
                        }
                    }
                    return true;
                }
                else
                {
                    buffer[i] = cmd;
                    i++;
                }
            }
            else
            {
                delayMicroseconds(10); // allow time to receive
            }
        }
        Serial.println(F("[buffer overflow]"));
    }
    return false;
}

void Parser::get_words (const String &command, std::vector<String> &words)
{
    int start = 0;
    bool in_word = false;
    for (int i = 0; i < command.length(); i++)
    {
        const char chr = command[i];
        if (isWhitespace(chr))
        {
            if (in_word)
            {
                words.push_back(command.substring(start, i));
                in_word = false;
            }
        }
        else if (!in_word)
        {
            in_word = true;
            start = i;
        }
        // else continue this word
    }
    if (in_word)
    {
        words.push_back(command.substring(start));
    }
}


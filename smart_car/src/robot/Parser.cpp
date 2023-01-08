/*
 * Parser.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Parser.hpp"

Parser::Parser (HardwareSerial& serial) : serial(serial)
{
}

void Parser::handle_command (Executor &executor)
{
    if (serial.available())
    {
        const int size = 256;
        char buffer[size];
        int i = 0;
        while (i < size)
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
                        //serial.print("execute: ");
                        //serial.println(command);
                        std::vector<String> words;
                        get_words(command, words);
                        if (!words.empty())
                        {
                            executor.execute_command(words);
                        }
                    }
                    return;
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
    }
}

void Parser::get_words (const String command, std::vector<String>& result)
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
                result.push_back(command.substring(start, i));
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
        result.push_back(command.substring(start));
    }
}


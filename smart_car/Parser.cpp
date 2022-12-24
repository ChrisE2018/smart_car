/*
 * Parser.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Parser.hpp"

Parser::Parser ()
{
}

void Parser::handle_command (Executor &executor)
{
//    if (false)
//    {
//        if (irrecv.decode(&ir_results))  // have we received an IR signal?
//        {
//            const unsigned long value = ir_results.value;
//            const char cmd = '.'; //translate_ir(value);
//            Serial.print(value);
//            Serial.print(" IR Command: ");
//            Serial.println(cmd);
//            switch (cmd)
//            {
//                case '<':
//                    car.reverse(SPEED_FULL, 500);
//                    break;
//                case '>':
//                    car.forward(SPEED_FULL, 500);
//                    break;
//                case '=':
//                    car.all_stop();
//                    break;
//                case 'd':
//                    car.turn_clockwise(SPEED_FULL, 500);
//                    break;
//                case 'u':
//                    car.turn_counterclockwise(SPEED_FULL, 500);
//                    break;
//            }
//            delay(500);
//            irrecv.resume();  // receive the next value
//        }
//    }
    if (Serial.available())
    {
        delay(1); // allow time to receive
        const int size = 256;
        char buffer[size];
        int i = 0;
        while (i < size)
        {
            if (Serial.available())
            {
                const char cmd = Serial.read();
                if (cmd == '\n' || cmd == '\r')
                {
                    buffer[i] = '\0';
                    String command(buffer);
                    Serial.print("execute_command: ");
                    Serial.println(command);
                    const int word_limit = 10;
                    String words[word_limit];
                    int n = get_words(command, words, word_limit);
                    if (n > 0)
                    {
                        executor.execute_command(n, words);
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
                delay(1); // allow time to receive
            }
        }
    }
}

int Parser::get_words (const String command, String result[], int max_words)
{
    int word = 0;
    int start = 0;
    bool in_word = false;
    for (int i = 0; i < command.length(); i++)
    {
        const char chr = command[i];
        if (isWhitespace(chr))
        {
            if (in_word)
            {
                result[word] = command.substring(start, i);
                word++;
                in_word = false;
                if (word >= max_words)
                    return word;
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
        result[word] = command.substring(start);
        word++;
    }
    return word;
}


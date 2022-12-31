/*
 * Logging.cpp
 *
 *  Created on: Dec 27, 2022
 *      Author: cre
 */

#include "Logging.hpp"
#include "Arduino.h"
#include <time.h>

Logging::Logging ()
{
}

Logger* Logging::get_logger (const String name)
{
    std::map<String, Logger *>::iterator it = loggers.find(name);
    if (it == loggers.end())
    {
        Logger* result = new Logger(*this, name);
        loggers[name] = result;
        return result;
    }
    return it->second;
}

void Logging::log (const Level level, const char *file, const int line, Logger *logger,
        const char *msg)
{
    for (int i = 0; i < format.length(); i++)
    {
        const char ch = format[i];
        if (ch != '%')
        {
            Serial.print(ch);
        }
        else
        {
            i++; // Increment loop control variable
            const char s = format[i];
            switch (s)
            {
                // https://www.tutorialspoint.com/log4j/log4j_patternlayout.htm
                // m = message; t = time; f = file; l = line; c = category; % = percent; n = newline
                case '%':
                    Serial.print(s);
                    break;
                case 'c':
                    Serial.print(logger->get_category());
                    break;
                case 'f':
                    Serial.print(file);
                    break;
                case 'l':
                    Serial.print(line);
                    break;
                case 'm':
                    Serial.print(msg);
                    break;
                case 'n':
                    Serial.println();
                    break;
                case 't':
                    Serial.print((double)millis(), 3);
                    break;
            }
        }
    }
}

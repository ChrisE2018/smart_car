/*
 * Logging.hpp
 *
 *  Created on: Dec 27, 2022
 *      Author: cre
 */

#pragma once

#include "Arduino.h"
#include "Logger.hpp"
#include <map>

class Logging
{
    public:
        Logging ();

        Logger* get_logger (const String name);

        void log (const Level level, const char *file, const int line, Logger *logger,
                const char *msg);

    private:
        std::map<String, Logger *> loggers;
        String format = "%t [%c] %m%n";
};


/*
 * Logger.hpp
 *
 *  Created on: Dec 27, 2022
 *      Author: cre
 */

#pragma once

class Logging;

enum Level
{
    NONE, WARN, INFO, DEBUG, ALL
};

#define loginfo(logger, format, args...) logger.log(__FILE__, __LINE__, INFO, format, args)
#define logwarn(logger, format, args...) logger.log(__FILE__, __LINE__, WARN, format, args)

class Logger
{
    public:
        Logger (Logging &logging, const String name);
        void set_level (Level level);
        bool is_enabled (Level level);
        void log (const char *file, int line, Level level, const char *format, ...);
        const String get_category ();

    private:
        Logging &logging;
        const String name;
        const int buffer_size = 256;
        Level level = INFO;
};


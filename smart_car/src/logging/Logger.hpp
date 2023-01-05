/*
 * Logger.hpp
 *
 *  Created on: Jan 2, 2023
 *      Author: cre
 */

#pragma once

#include <Arduino.h>
#include <vector>

class Appender;

enum class Level
{
    error, warning, info, debug
};

const char* stringify (const Level level);

#define LOG_ERROR(logger, fmt, args...) logger.logging(Level::error, __LINE__, fmt, args);
#define LOG_WARNING(logger, fmt, args...) logger.logging(Level::warning, __LINE__, fmt, args);
#define LOG_INFO(logger, fmt, args...) logger.logging(Level::info, __LINE__, fmt, args);
#define LOG_DEBUG(logger, fmt, args...) logger.logging(Level::debug, __LINE__, fmt, args);

class Logger
{
    public:
        static Logger *ROOT;
        Logger (const String name);
        Logger (Logger *parent, const String name, const Level level);
        const String get_name () const;
        const String get_short_name () const;
        void set_level (const Level level);
        const Level get_level () const;
        void add_appender (Appender *appender);
        void logging (const Level level, const int line, const char *format, ...);

    private:
        const unsigned int buffer_size = 256;
        Logger *parent;
        const String name;
        const String short_name;
        Level level;

        std::vector<Appender*> appenders;
        const String shorten (const String name);
        void append (const Logger *logger, const Level level, const int line, const char *message);
};

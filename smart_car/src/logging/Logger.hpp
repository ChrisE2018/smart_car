/*
 * Logger.hpp
 *
 *  Created on: Jan 2, 2023
 *      Author: cre
 */

#pragma once

#include <Arduino.h>
#include <vector>
#include "LogBuffer.hpp"

class Appender;

enum class Level
{
    error,
    warning,
    info,
    debug
};

const char* stringify (const Level level);

#define LOG_ERROR(logger, fmt, args...) logger.logging_p(Level::error, __LINE__, (const char *)F(fmt), args);
#define LOG_WARNING(logger, fmt, args...) logger.logging_p(Level::warning, __LINE__, (const char *)F(fmt), args);
#define LOG_INFO(logger, fmt, args...) logger.logging_p(Level::info, __LINE__, (const char *)F(fmt), args);
#define LOG_DEBUG(logger, fmt, args...) logger.logging_p(Level::debug, __LINE__, (const char *)F(fmt), args);

class Logger
{
    public:
        static Logger *ROOT;
        Logger (const String name);
        Logger (const String name, const Level level);
        Logger (Logger *parent, const String name, const Level level);
        const String get_name () const;
        const String get_short_name () const;
        const Level get_level () const;
        void add_appender (Appender *const appender);
        LogBuffer& info ();
        LogBuffer& debug ();
        LogBuffer& info (const int line);
        LogBuffer& debug (const int line);
        void logging (const Level level, const int line, const char *format, ...);
        void logging_p (const Level level, const int line, const char *format, ...);
        void append (const Logger *logger, const Level level, const int line, const char *message);

    private:
        static const int buffer_size = 256;
        static char buffer[buffer_size];
        Logger *const parent;
        const String name;
        const String short_name;
        const Level level;
        LogBuffer info_stream;
        LogBuffer debug_stream;

        std::vector<Appender*> appenders;
        const String shorten (const String name);
};


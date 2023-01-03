/*
 * Logger.hpp
 *
 *  Created on: Jan 2, 2023
 *      Author: cre
 */

#pragma once

#include <Arduino.h>

enum class Level
{
    error, warning, info, debug
};

const char* stringify(const Level level);

#define LOG_ERROR(logger, fmt, args...) logger.logging(Level::error, __LINE__, fmt, args);
#define LOG_WARNING(logger, fmt, args...) logger.logging(Level::warning, __LINE__, fmt, args);
#define LOG_INFO(logger, fmt, args...) logger.logging(Level::info, __LINE__, fmt, args);
#define LOG_DEBUG(logger, fmt, args...) logger.logging(Level::debug, __LINE__, fmt, args);

class Logger
{
    private:
        const unsigned int buffer_size = 256;
        const String name;
        Level level;

    public:
        Logger (const String name);
        Logger (const String name, const Level level);
        const String get_name () const;
        void set_level (const Level level);
        const Level get_level () const;
        void logging (const Level level, const int line, const char *format, ...);
};


/*
 * LogBuffer.hpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#pragma once

#include <streambuf>
#include <iostream>
#include <string>

class Logger;
enum class Level;

class LogBuffer : private std::streambuf, public std::ostream
{
    public:
        LogBuffer (Logger *logger, const Level level);
        std::string get_buffer ();
        void reset ();

    protected:
        virtual std::streambuf::int_type overflow (std::streambuf::int_type c) override;

    private:
        static const int buffer_size = 255;
        char buffer[buffer_size + 1];
        int pos = 0;
        Logger *logger;
        const Level level;

        void flush ();

};


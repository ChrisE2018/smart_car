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
#include <WString.h>

class Logger;
enum class Level
;

std::ostream& operator<< (std::ostream &lhs, const __FlashStringHelper *pstr);

// http://www.angelikalanger.com/Articles/C++Report/IOStreamsDerivation/IOStreamsDerivation.html
// https://gcc.gnu.org/onlinedocs/libstdc++/manual/streambufs.html

class LogBuffer : private std::streambuf, public std::ostream
{
    public:
        LogBuffer (Logger *logger, const Level level);
        void reset ();
        void set_line(const int line);

    protected:
        virtual std::streambuf::int_type overflow (const std::streambuf::int_type c) override;

    private:
        static const int buffer_size = 128;
        static char buffer[buffer_size];
        static int pos;
        Logger *const logger;
        const Level level;
        int line = 0;

        void flush ();
};

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
#include "Level.hpp"

class Logger;

std::ostream& operator<< (std::ostream &lhs, const __FlashStringHelper *pstr);

// http://www.angelikalanger.com/Articles/C++Report/IOStreamsDerivation/IOStreamsDerivation.html
// https://gcc.gnu.org/onlinedocs/libstdc++/manual/streambufs.html
class LogBuffer: private std::streambuf, public std::ostream
{
    public:
        LogBuffer ();
        void set_logger (Logger *const logger, const Level level, const int line);

    protected:
        virtual std::streambuf::int_type overflow (const std::streambuf::int_type c) override;

    private:
        Logger *logger;
        Level level;
        int line = 0;
        void flush ();
        void reset ();
};

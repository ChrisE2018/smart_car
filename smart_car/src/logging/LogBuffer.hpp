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

class LogBuffer : public std::streambuf
{
    public:
        LogBuffer ();
        std::string get_buffer ();
        void reset ();

    protected:
        virtual int_type overflow (int_type c) override;

    private:
        static const int buffer_size = 255;
        char buffer[buffer_size + 1];
        int pos = 0;

        virtual void flush ();

};


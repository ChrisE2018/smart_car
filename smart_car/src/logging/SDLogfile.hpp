/*
 * SDLogfile.hpp
 *
 *  Created on: Jan 3, 2023
 *      Author: cre
 */

#pragma once

#include <WString.h>

class SDLogfile
{
    public:
        SDLogfile ();
        void open();
        void close();
        void print(String text);
        void print(char* text);
};


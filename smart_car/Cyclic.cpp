/*
 * Cyclic.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Arduino.h"
#include "Cyclic.hpp"

Cyclic::Cyclic ()
{
}

Cyclic::~Cyclic ()
{
}

Mode Cyclic::get_mode ()
{
    return mode;
}

void Cyclic::set_mode (Mode _mode)
{
    mode = _mode;
    Serial.print("Cyclic mode: ");
    Serial.println(_mode);
}

void Cyclic::cycle ()
{
}

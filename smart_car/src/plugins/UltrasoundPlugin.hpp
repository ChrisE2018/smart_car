/*
 * UltrasoundPlugin.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#pragma once

#include "Plugin.hpp"
#include <SR04.h>

const int ULTRASOUND_TRIGGER = 12;  // blue
const int ULTRASOUND_ECHO = 11;     // green

class UltrasoundPlugin: public Plugin
{
    public:
        UltrasoundPlugin ();
        long get_distance ();
        void print_distance ();
        void cycle () override;

    private:
        SR04 sr04;
};


/*
 * UltrasoundPlugin.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#pragma once

#include <Arduino.h>
#include "Plugin.hpp"
#include <SR04.h>

const int ULTRASOUND_TRIGGER = 12;  // blue
const int ULTRASOUND_ECHO = 11;     // green

class Car;

class UltrasoundPlugin: public Plugin
{
    public:
        UltrasoundPlugin (Car &car);
        virtual int get_preferred_interval () const;
        virtual int get_expected_ms () const;
        long get_distance ();
        void cycle () override;

    private:
        Car &car;
        SR04 sr04;
};


/*
 * Copyright (c) 2023 by Christopher Eliot.
 *
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
        static constexpr int READING = 2;
        static constexpr int BLOCKING = 3;
        UltrasoundPlugin (Car &car);
        virtual int get_preferred_interval () const;
        virtual int get_expected_us () const;
        long get_distance () const;
        virtual void cycle () override;
        virtual void trace () override;

    private:
        Car &car;
        SR04 sr04;
        long distance = 0;
        static constexpr int blocking_distance = 10; // centimeters
};


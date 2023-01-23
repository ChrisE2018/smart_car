/*
 * Motor.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include <Arduino.h>
#include "Plugin.hpp"
#include <ostream>

constexpr int SPEED_FULL = 255;
constexpr int SPEED_Q3 = 196;
constexpr int SPEED_175 = 175;
constexpr int SPEED_160 = 160;
constexpr int SPEED_150 = 150;
constexpr int SPEED_HALF = 128;
constexpr int SPEED_100 = 100;
constexpr int SPEED_Q1 = 64;
constexpr int SPEED_CRAWL = 32;
constexpr int SPEED_STOP = 0;

constexpr int DISABLED_LED = 99;

enum MotorDirection : unsigned char
{
    STOP, FORWARD, REVERSE
};

const std::string stringify (const MotorDirection id);
std::ostream& operator<< (std::ostream &lhs, MotorDirection direction);

enum MotorLocation : unsigned char
{
    RIGHT_FRONT, LEFT_FRONT, RIGHT_REAR, LEFT_REAR
};

const std::string stringify (const MotorLocation id);
std::ostream& operator<< (std::ostream &lhs, MotorLocation location);

class MotorPlugin: public Plugin
{
    public:
        MotorPlugin (const PluginId id, const MotorLocation location, int enable, int forward, int reverse,
                int forward_led, int reverse_led);

        friend std::ostream& operator<< (std::ostream &lhs, const MotorPlugin &motor);
        bool setup ();
        virtual bool is_cyclic () const;
        void led_demo (const int duration) const;
        MotorLocation get_location () const;
        MotorDirection get_direction () const;
        int get_speed () const;
        void set_speed (const int speed);
        void set_limit (const unsigned long limit, const int value);
        void set_delta_limit (const unsigned long delta, const int value);

    private:
        const MotorLocation location;
        const int enable_pin;
        const int forward_pin;
        const int reverse_pin;
        const int forward_led;
        const int reverse_led;
        MotorDirection direction = MotorDirection::STOP;
        int speed = 0;

        void drive_forward (const int speed);
        void drive_reverse (const int speed);
        void drive_zero_speed ();
};


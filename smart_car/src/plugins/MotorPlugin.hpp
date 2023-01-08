/*
 * Motor.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include "Arduino.h"
#include "Plugin.hpp"
#include <ostream>

const int SPEED_FULL = 255;
const int SPEED_Q3 = 196;
const int SPEED_175 = 175;
const int SPEED_160 = 160;
const int SPEED_150 = 150;
const int SPEED_HALF = 128;
const int SPEED_100 = 100;
const int SPEED_Q1 = 64;
const int SPEED_CRAWL = 32;
const int SPEED_STOP = 0;

enum MotorDirection
{
    STOP, FORWARD, REVERSE
};

std::ostream& operator<< (std::ostream &lhs, MotorDirection direction);

enum MotorLocation
{
    RIGHT = 0, LEFT = 1
};

std::ostream& operator<< (std::ostream &lhs, MotorLocation location);

class MotorPlugin: public Plugin
{
    public:
        MotorPlugin (const PluginId id, const MotorLocation location, int enable, int forward,
                int reverse, int speed_counter_pin, int forward_led, int reverse_led) : location(
                location), enable_pin(enable), forward_pin(forward), reverse_pin(reverse), speed_counter_pin(
                speed_counter_pin), forward_led(forward_led), reverse_led(reverse_led), Plugin(id)
        {
        }

        bool setup ();
        void led_demo (const int duration) const;
        void drive_forward (const int speed);
        void drive_reverse (const int speed);
        void drive_stop ();
        MotorLocation get_location () const;
        MotorDirection get_direction () const;
        int get_speed () const;
        float get_velocity () const;
        unsigned long get_speed_counter () const;
        float get_speed_counter_velocity (const unsigned long now);
        void set_desired_velocity (float desired_velocity);
        virtual int get_preferred_interval () const;
        virtual int get_expected_ms () const;
        void cycle ();

    private:

        // 55 mm wheels
        // 20 encoder slots per revolution
        // Multiply by one million to convert micros to seconds
        // meters-per-micro = PI *diameter / encoder_slots
        const double count_to_meters_per_second = 1000.0 * 1000.0 * M_PI * 0.055 / 20.0;

        const MotorLocation location;
        const int enable_pin;
        const int forward_pin;
        const int reverse_pin;
        const int speed_counter_pin;
        const int forward_led;
        const int reverse_led;
        MotorDirection direction = STOP;
        int speed = 0;
        unsigned long speed_counter_checkpoint = 0;

        float desired_velocity = 0;
        unsigned long last_cycle_micros = 0;
        float last_cycle_error = 0;
        float cumulative_velocity_error = 0;
        float cumulative_error_time = 0;
};

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

enum MotorDirection : unsigned char
{
    STOP, FORWARD, REVERSE
};

const std::string stringify (const MotorDirection id);
std::ostream& operator<< (std::ostream &lhs, MotorDirection direction);

enum MotorLocation : unsigned char
{
    RIGHT = 0, LEFT = 1
};

const std::string stringify (const MotorLocation id);
std::ostream& operator<< (std::ostream &lhs, MotorLocation location);

class MotorPlugin: public Plugin
{
    public:
        MotorPlugin (const PluginId id, const MotorLocation location, int enable, int forward, int reverse,
                int speed_counter_pin, int forward_led, int reverse_led) :
                location(location), enable_pin(enable), forward_pin(forward), reverse_pin(reverse), speed_counter_pin(
                        speed_counter_pin), forward_led(forward_led), reverse_led(reverse_led), Plugin(id)
        {
        }

        friend std::ostream& operator<< (std::ostream &lhs, const MotorPlugin &motor);
        bool setup ();
        void led_demo (const int duration) const;
        void drive_forward (const int speed);
        void drive_reverse (const int speed);
        void drive_stop ();
        MotorLocation get_location () const;
        MotorDirection get_direction () const;
        int get_speed () const;
        void set_speed (const int speed);
        unsigned long get_speed_counter () const;
        float get_measured_velocity () const;
        float get_desired_velocity () const;
        void set_desired_velocity (const float desired_velocity);
        void cancel_auto_velocity ();
        float get_velocity_error () const;
        virtual int get_preferred_interval () const override;
        virtual int get_expected_us () const override;
        void cycle () override;

    private:
        const MotorLocation location;
        const int enable_pin;
        const int forward_pin;
        const int reverse_pin;
        const int speed_counter_pin;
        const int forward_led;
        const int reverse_led;
        MotorDirection direction = MotorDirection::STOP;
        int speed = 0;

        // Support for speed encoders
        static constexpr int minimum_cycle_ms = 100;
        static constexpr int minimum_speed_ticks = 5;

        // 55 mm wheels
        // 20 encoder slots per revolution
        // meters-per-micro = PI *diameter / encoder_slots
        static constexpr double count_to_meters_per_second = M_PI * 0.055 / 20.0;

        static constexpr float k0 = 0.45 * SPEED_FULL;
        static constexpr float k1 = 0.15; //0.30;
        static constexpr float k2 = -0.01;
        static constexpr float k3 = 0; //0.15;
        static constexpr float k4 = 0; //0.2;
        bool auto_velocity = false;
        float desired_velocity = 0;
        float measured_velocity = 0;
        unsigned long last_cycle_ms = 0;
        long speed_counter = 0;
        float velocity_error = 0;
        float cumulative_velocity_error = 0;
        void drive_zero_speed ();

        // Only returns +1 or -1 unlike signum
        int sign (const float direction) const;
};


/*
 * Car.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include <arduino.h>
#include <vector>
#include "Logger.hpp"
#include "../plugins/MotorPlugin.hpp"
#include "Mode.hpp"
#include "Parser.hpp"
#include "board_pins.hpp"

class Car: public Executor
{
    public:
        Car ();
        friend std::ostream& operator<< (std::ostream &os, const Car &car);

        void setup ();
        void set_mode (const Mode mode);
        void cycle ();
        void demo_drive_leds ();
        virtual void execute_command (const std::vector<String> &words) override;

        void all_stop ();
        void drive_stop (const MotorLocation motor);
        void set_speed (const MotorLocation motor, const int speed);
        void set_right_speed (const int speed);
        void set_left_speed (const int speed);
        int get_drive_speed (const MotorLocation motor) const;
        float get_desired_velocity (const MotorLocation motor) const;
        void set_desired_velocity (const MotorLocation motor, const float velocity);
        float get_measured_velocity (const MotorLocation motor) const;
        float get_cumulative_velocity_error (const MotorLocation motor) const;

    private:
        logging::Logger logger;
        unsigned long cycle_count = 0;
        unsigned long total_cycle_us = 0;

        Parser serial_parser;
        Parser bluetooth_parser;

        Mode mode = Mode::COMMAND_MODE;

        void command_cycle ();
        void help_command ();
};


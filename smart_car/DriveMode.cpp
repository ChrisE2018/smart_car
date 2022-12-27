/*
 * DriveMode.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "Car.hpp"
#include "DriveMode.hpp"

DriveMode::DriveMode (PluginId id, Car &car, const int duration,
        const MotorDirection right_motor_direction, const MotorDirection left_motor_direction) : Plugin(
        id), car(car), duration(duration), right_motor_direction(right_motor_direction), left_motor_direction(
        left_motor_direction)
{
}

void DriveMode::set_enabled (const bool enable)
{
    Plugin::set_enabled(enable);
    if (enable)
    {
        deadline = millis() + duration;
        Serial.print("Starting DriveMode: ");
        Serial.print(get_id());
        Serial.print(" at ");
        Serial.print(right_motor_speed);
        Serial.print(", ");
        Serial.print(left_motor_speed);
        Serial.print(" for ");
        Serial.print(duration);
        Serial.print(" until ");
        Serial.println(deadline);
        switch (right_motor_direction)
        {
            case STOP:
                car.drive_stop(RIGHT);
                break;
            case FORWARD:
                car.drive_forward(RIGHT, right_motor_speed);
                break;
            case REVERSE:
                car.drive_reverse(RIGHT, right_motor_speed);
                break;
        }
        switch (left_motor_direction)
        {
            case STOP:
                car.drive_stop(LEFT);
                break;
            case FORWARD:
                car.drive_forward(LEFT, left_motor_speed);
                break;
            case REVERSE:
                car.drive_reverse(LEFT, left_motor_speed);
                break;
        }
    }
}

void DriveMode::cycle ()
{
    if (is_enabled())
    {
        if (deadline < millis())
        {
            Serial.print("Stopping DriveMode: ");
            Serial.println(get_id());
            car.all_stop();
            set_enabled(false);
        }
    }
}

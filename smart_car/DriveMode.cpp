/*
 * DriveMode.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "DriveMode.hpp"
#include "Car.hpp"

DriveMode::DriveMode (Car &car, Mode mode, const int duration,
        const MotorDirection right_motor_direction, const MotorDirection left_motor_direction) : car(
        car), mode(mode), duration(duration), right_motor_direction(right_motor_direction), left_motor_direction(
        left_motor_direction)
{

}

void DriveMode::set_mode (Mode new_mode)
{
    Cyclic::set_mode(new_mode);
    if (mode == new_mode)
    {
        deadline = millis() + duration;
        Serial.print("Starting DriveMode: ");
        Serial.print(mode);
        Serial.print(" for ");
        Serial.print(duration);
        Serial.print(" until ");
        Serial.println(deadline);
        switch (right_motor_direction)
        {
            case STOP:
                car.drive_stop(0);
                break;
            case FORWARD:
                car.drive_forward(0, right_motor_speed);
                break;
            case REVERSE:
                car.drive_reverse(0, right_motor_speed);
                break;
        }
        switch (left_motor_direction)
        {
            case STOP:
                car.drive_stop(1);
                break;
            case FORWARD:
                car.drive_forward(1, left_motor_speed);
                break;
            case REVERSE:
                car.drive_reverse(1, left_motor_speed);
                break;
        }
    }
}

void DriveMode::cycle ()
{
    if (get_mode() == mode)
    {
        if (deadline < millis())
        {
            Serial.print("Stopping DriveMode: ");
            Serial.println(mode);
            car.all_stop();
            car.set_mode(COMMAND_MODE);
        }
    }
}

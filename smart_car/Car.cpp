/*
 * Car.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Car.hpp"

Car::Car () : parser(*this)
{
}

void Car::setup ()
{
    for (int motor = 0; motor < MOTOR_COUNT; motor++)
    {
        motors[motor].setup();
    }
}
void Car::demo_drive_leds ()
{
    int duration = 150;
    for (int i = 0; i < 3; i++)
    {
        for (int motor = 0; motor < MOTOR_COUNT; motor++)
        {
            motors[motor].led_demo(duration);
        }
        duration /= 2;
    }
}
void Car::all_stop ()
{
    for (int motor = 0; motor < MOTOR_COUNT; motor++)
    {
        motors[motor].drive_stop();
    }
}

void Car::drive_stop (int motor)
{
    motors[motor].drive_stop();
}

void Car::drive_forward (const int motor, const int speed)
{
    motors[motor].drive_forward(speed);
}

void Car::drive_reverse (const int motor, int speed)
{
    motors[motor].drive_reverse(speed);
}

void Car::forward (const int speed, const int duration)
{
    Serial.print("forward ");
    Serial.print(speed);
    Serial.print(" for ");
    Serial.print(duration);
    Serial.println(" ms");
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        drive_forward(i, speed);
        delay(10);
    }
    delay(duration);
    Serial.println("stop");
    all_stop();
}

void Car::reverse (const int speed, const int duration)
{
    Serial.print("reverse ");
    Serial.print(speed);
    Serial.print(" for ");
    Serial.print(duration);
    Serial.println(" ms");
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        drive_reverse(i, speed);
        delay(10);
    }
    delay(duration);
    Serial.println("stop");
    all_stop();
}

void Car::turn_clockwise (const int speed, const int duration)
{
    Serial.print("turn clockwise ");
    Serial.print(speed);
    Serial.print(" for ");
    Serial.print(duration);
    Serial.println(" ms");

    drive_forward(0, SPEED_FULL);
    drive_reverse(1, SPEED_FULL);

    delay(duration);
    Serial.println("stop");
    all_stop();
}

void Car::turn_counterclockwise (const int speed, const int duration)
{
    Serial.print("turn counterclockwise ");
    Serial.print(speed);
    Serial.print(" for ");
    Serial.print(duration);
    Serial.println(" ms");

    drive_reverse(0, SPEED_FULL);
    drive_forward(1, SPEED_FULL);

    delay(duration);
    Serial.println("stop");
    all_stop();
}

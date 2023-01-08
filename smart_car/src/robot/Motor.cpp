/*
 * Motor.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Motor.hpp"
#include "smart_car.hpp"

unsigned long int speed_counter_right = 0;
unsigned long int speed_counter_left = 0;

void isr_right ()
{
    speed_counter_right++;
}
void isr_left ()
{
    speed_counter_left++;
}

std::ostream& operator<< (std::ostream &lhs, MotorDirection direction)
{
    switch (direction)
    {
        case STOP:
            lhs << "STOP";
            break;
        case FORWARD:
            lhs << "FORWARD";
            break;
        case REVERSE:
            lhs << "REVERSE";
            break;
    }
    return lhs;
}

std::ostream& operator<< (std::ostream &lhs, MotorLocation location)
{
    switch (location)
    {
        case RIGHT:
            lhs << "RIGHT";
            break;
        case LEFT:
            lhs << "LEFT";
            break;
    }
    return lhs;
}

void Motor::setup ()
{
    pinMode(enable_pin, OUTPUT);
    pinMode(forward_pin, OUTPUT);
    pinMode(reverse_pin, OUTPUT);
    pinMode(forward_led, OUTPUT);
    pinMode(reverse_led, OUTPUT);
    drive_stop();
    void (*isr) () = (location == RIGHT) ? isr_right : isr_left;
    attachInterrupt(digitalPinToInterrupt(speed_counter_pin), isr, RISING);
}

void Motor::led_demo (const int duration) const
{
    digitalWrite(forward_led, HIGH);
    delay(duration);
    digitalWrite(forward_led, LOW);
    delay(duration);
    digitalWrite(reverse_led, HIGH);
    delay(duration);
    digitalWrite(reverse_led, LOW);
    delay(duration);
}

void Motor::drive_forward (const int _speed)
{
    if (direction != FORWARD || speed != _speed)
    {
        speed = _speed;
        direction = FORWARD;
        digitalWrite(forward_led, HIGH);
        digitalWrite(reverse_led, LOW);
        digitalWrite(reverse_pin, LOW);
        digitalWrite(forward_pin, HIGH);
        analogWrite(enable_pin, _speed);
        cout << "drive " << location << " " << direction << " at " << speed << std::endl;
    }
}

void Motor::drive_reverse (const int _speed)
{
    if (direction != REVERSE || speed != _speed)
    {
        speed = _speed;
        direction = REVERSE;
        digitalWrite(forward_led, LOW);
        digitalWrite(reverse_led, HIGH);
        digitalWrite(forward_pin, LOW);
        digitalWrite(reverse_pin, HIGH);
        analogWrite(enable_pin, _speed);
        cout << "drive " << location << " " << direction << " at " << speed << std::endl;
    }
}

void Motor::drive_stop ()
{
    if (direction != STOP || speed != 0)
    {
        direction = STOP;
        speed = 0;
        desired_velocity = 0;
        digitalWrite(forward_led, LOW);
        digitalWrite(reverse_led, LOW);
        digitalWrite(forward_pin, LOW);
        digitalWrite(reverse_pin, LOW);
        analogWrite(enable_pin, LOW);
        cout << "drive " << location << " " << direction << " at " << speed << std::endl;
    }
}

MotorLocation Motor::get_location () const
{
    return location;
}

MotorDirection Motor::get_direction () const
{
    return direction;
}

int Motor::get_speed () const
{
    return speed;
}

float Motor::get_velocity () const
{
    if (direction == REVERSE)
    {
        return -(speed / 256.0);
    }
    else
    {
        return (speed / 256.0);
    }
}

unsigned long Motor::get_speed_counter () const
{
    return (location == RIGHT) ? speed_counter_right : speed_counter_left;
}

float Motor::get_speed_counter_velocity (const unsigned long now)
{
    unsigned long count = 0;
    if (location == RIGHT)
    {
        count = speed_counter_right;
        speed_counter_right = 0;
    }
    else
    {
        count = speed_counter_left;
        speed_counter_left = 0;
    }
    const unsigned long checkpoint = speed_counter_checkpoint;
    const double duration = now - checkpoint; // microseconds
    speed_counter_checkpoint = now;
    if (duration > 0)
    {
        return count * count_to_meters_per_second / duration;
    }
    return 0;
}

void Motor::set_desired_velocity (float _desired_velocity)
{
    desired_velocity = _desired_velocity;
}

void Motor::cycle ()
{
    unsigned long now = micros();
    unsigned long delta_time = now - last_cycle_micros;
    const float measured_velocity = get_speed_counter_velocity(now);
    const float velocity_error = desired_velocity - measured_velocity;
    const float velocity_error_rate = (last_cycle_error - velocity_error) / delta_time;
    cumulative_velocity_error += velocity_error;
    cumulative_error_time += delta_time;
    last_cycle_error = velocity_error;
    last_cycle_micros = now;
    const float k0 = 3;
    const float k1 = 2;
    const float k2 = 1;
    const float control = k0
            * (velocity_error + k1 * cumulative_velocity_error / cumulative_error_time
                    + k2 * velocity_error_rate);

    cout << "Motor " << location << " control  " << control << std::endl;
    // set_velocity(control);
}

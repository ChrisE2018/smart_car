/*
 * Car.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Car.hpp"

#include "DemoPlugin.hpp"
#include "DrivePlugin.hpp"
#include "WallPlugin.hpp"

Car::Car () : sr04(ULTRASOUND_ECHO, ULTRASOUND_TRIGGER), parser(Serial), parser1(Serial1)
{
    wall_mode = new WallPlugin(*this);
    demo_mode = new DemoPlugin(*this);
    forward_mode = new DrivePlugin(FORWARD_PLUGIN, *this, 500, FORWARD, FORWARD);
    reverse_mode = new DrivePlugin(REVERSE_PLUGIN, *this, 500, REVERSE, REVERSE);
    clockwise_mode = new DrivePlugin(CLOCKWISE_PLUGIN, *this, 500, FORWARD, REVERSE);
    counterclockwise_mode = new DrivePlugin(COUNTERCLOCKWISE_PLUGIN, *this, 500, REVERSE, FORWARD);
    plugins.push_back(wall_mode);
    plugins.push_back(demo_mode);
    plugins.push_back(forward_mode);
    plugins.push_back(reverse_mode);
    plugins.push_back(clockwise_mode);
    plugins.push_back(counterclockwise_mode);
}

Car::~Car ()
{
    for (Plugin *plugin : plugins)
    {
        delete plugin;
    }
    plugins.clear();
}

void Car::setup ()
{
    for (int motor = 0; motor < MOTOR_COUNT; motor++)
    {
        motors[motor].setup();
    }
    setup_imu();
}

void Car::setup_imu ()
{
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    Serial.println("imu setup");
}

void Car::set_mode (Mode _mode)
{
    Serial.print("Setting ");
    Serial.print(plugins.size());
    Serial.print(" plugin modes: ");
    Serial.println(_mode);
    mode = _mode;
    for (Plugin *plugin : plugins)
    {
        plugin->set_enabled(false);
    }
    for (Plugin *plugin : plugins)
    {
        plugin->set_mode(_mode);
    }
}

void Car::cycle ()
{
    const long cycle_start_us = micros();

    handle_command();
    if (show_distance)
    {
        print_distance();
    }
    if (show_imu)
    {
        read_imu();
    }
    for (Plugin *plugin : plugins)
    {
        plugin->cycle();
    }

    total_cycle_us += micros() - cycle_start_us;
    cycle_count++;
}

long Car::get_distance ()
{
    return sr04.Distance();
}

void Car::print_distance ()
{
    long d = sr04.Distance();
    Serial.print("Distance ");
    Serial.print(d);
    Serial.println(" cm");
}

void Car::read_imu ()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
    AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    Serial.print(" | AcX = "); // Accelerator
    Serial.print(AcX);
    Serial.print(" | AcY = ");
    Serial.print(AcY);
    Serial.print(" | AcZ = ");
    Serial.print(AcZ);
    Serial.print(" | Tmp = ");
    Serial.print(Tmp / 340.00 + 36.53);  //equation for temperature in degrees C from datasheet
    Serial.print(" | GyX = "); // Gyro
    Serial.print(GyX);
    Serial.print(" | GyY = ");
    Serial.print(GyY);
    Serial.print(" | GyZ = ");
    Serial.print(GyZ);
    Serial.println();
}

void Car::handle_command ()
{
    parser.handle_command(*this);
    parser1.handle_command(*this);
}

/** Execute a command from a buffer.
 @param command The full command string with no newline.
 */
void Car::execute_command (const int n, const String words[])
{
    String command = words[0];
    Serial.print("Command: ");
    Serial.println(command);
    if (command == "b")
    {
        int speed = SPEED_FULL;
        int duration = 500;
        if (n > 1)
        {
            speed = words[1].toInt();
        }
        if (n > 2)
        {
            duration = words[2].toInt();
        }
        set_mode(COMMAND_MODE);
        reverse_mode->set_duration(duration);
        reverse_mode->set_right_speed(speed);
        reverse_mode->set_left_speed(speed);
        reverse_mode->set_enabled(true);
    }
    else if (command == "c")
    {
        Serial.println("Current mode is COMMAND MODE");
        set_mode(COMMAND_MODE);
    }
    else if (command == "demo")
    {
        Serial.println("Current mode is DEMO_MODE MODE");
        set_mode(DEMO_MODE);
    }
    else if (command == "wall")
    {
        Serial.println("Current mode is WALL_MODE MODE");
        set_mode(WALL_MODE);
    }
    else if (command == "distance")
    {
        show_distance = !show_distance;
    }
    else if (command == "imu")
    {
        show_imu = !show_imu;
    }
    else if (command == "led")
    {
        demo_drive_leds();
    }
    else if (command == "f")
    {
        int speed = SPEED_FULL;
        int duration = 500;
        if (n > 1)
        {
            speed = words[1].toInt();
        }
        if (n > 2)
        {
            duration = words[2].toInt();
        }
        set_mode(COMMAND_MODE);
        forward_mode->set_duration(duration);
        forward_mode->set_right_speed(speed);
        forward_mode->set_left_speed(speed);
        forward_mode->set_enabled(true);
    }
    else if (command == "l")
    {
        int speed = SPEED_FULL;
        int duration = 500;
        if (n > 1)
        {
            speed = words[1].toInt();
        }
        if (n > 2)
        {
            duration = words[2].toInt();
        }
        set_mode(COMMAND_MODE);
        clockwise_mode->set_duration(duration);
        clockwise_mode->set_right_speed(speed);
        clockwise_mode->set_left_speed(speed);
        clockwise_mode->set_enabled(true);
    }
    else if (command == "r")
    {
        int speed = SPEED_FULL;
        int duration = 500;
        if (n > 1)
        {
            speed = words[1].toInt();
        }
        if (n > 2)
        {
            duration = words[2].toInt();
        }
        set_mode(COMMAND_MODE);
        counterclockwise_mode->set_duration(duration);
        counterclockwise_mode->set_right_speed(speed);
        counterclockwise_mode->set_left_speed(speed);
        counterclockwise_mode->set_enabled(true);
    }
    else if (command == "s")
    {
        all_stop();
        set_mode(COMMAND_MODE);
    }
    else if (command == "?")
    {
        help_command();
    }
    else
    {
        Serial.print("Invalid command: ");
        Serial.println(command);
    }
}

void Car::help_command ()
{
    Serial.println("b - backward");
    Serial.println("c - command mode");
    Serial.println("d - demo mode");
    Serial.println("f - forward");
    Serial.println("l - left turn");
    Serial.println("r - right turn");
    Serial.println("s - stop moving");
    Serial.println("? - help");
    if (mode == COMMAND_MODE)
    {
        Serial.println("Current mode is COMMAND MODE");
    }
    else if (mode == DEMO_MODE)
    {
        Serial.println("Current mode is DEMO MODE");
    }
    else if (mode == WALL_MODE)
    {
        Serial.println("Current mode is WALL_MODE");
    }
    else
    {
        Serial.print("Current mode is UNKNOWN MODE: ");
        Serial.println(mode);
    }
    read_imu();
    print_distance();

    Serial.print("Plugins: ");
    Serial.println(plugins.size());
    Serial.print("Cycle Count: ");
    Serial.print(cycle_count);
    Serial.print(" total cycle micros ");
    Serial.print(total_cycle_us);
    Serial.print(" average micros per cycle ");
    Serial.println(total_cycle_us / (double) cycle_count);
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

void Car::drive_stop (const MotorLocation motor)
{
    motors[motor].drive_stop();
}

void Car::drive_forward (const MotorLocation motor, const int speed)
{
    motors[motor].drive_forward(speed);
}

void Car::drive_reverse (const MotorLocation motor, const int speed)
{
    motors[motor].drive_reverse(speed);
}

WallPlugin* Car::get_wall_mode ()
{
    return wall_mode;
}

DemoPlugin* Car::get_demo_mode ()
{
    return demo_mode;
}

DrivePlugin* Car::get_forward_mode ()
{
    return forward_mode;
}

DrivePlugin* Car::get_reverse_mode ()
{
    return reverse_mode;
}

DrivePlugin* Car::get_clockwise_mode ()
{
    return clockwise_mode;
}

DrivePlugin* Car::get_counterclockwise_mode ()
{
    return counterclockwise_mode;
}

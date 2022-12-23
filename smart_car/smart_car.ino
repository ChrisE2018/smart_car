#include "Arduino.h"
#include <SR04.h>
#include "IRremote.h"
#include <MPU6050.h>
#include <Wire.h>
#include "Car.hpp"
#include "Motor.hpp"

// Program for robot car

void setup_imu ();
void handle_command ();
void print_distance ();
void read_imu ();
void command_mode_command ();
void demo_mode_command ();

void help_command ();

Car car;

const int ULTRASOUND_TRIGGER = 12;  // blue
const int ULTRASOUND_ECHO = 11;     // green

SR04 sr04 = SR04(ULTRASOUND_ECHO, ULTRASOUND_TRIGGER);

const int COMMAND_MODE = 0;
const int DEMO_MODE = 1;
int mode = COMMAND_MODE;

bool show_distance = false;

/*-----Infrared remote-----*/
const int ir_receiver_pin = 30;  // Signal Pin of IR receiver to Arduino Digital Pin 11
IRrecv irrecv(ir_receiver_pin);  // create instance of 'irrecv'
decode_results ir_results;       // create instance of 'decode_results'

/* IMU */

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

/* Control program. */

void setup ()
{
    Serial.begin(9600);
    Serial.println("Smart car");
    car.setup();
    car.demo_drive_leds();
    //Serial.println("IR Receiver Button Decode");
    //irrecv.enableIRIn();  // Start the receiver

    //setup_imu();
    Serial.println("Ready");
}

void setup_imu ()
{
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    Serial.println("imu setup");
}

// TODO Event based

void loop ()
{
    handle_command();

    if (show_distance)
    {
        print_distance();
    }
    if (mode == DEMO_MODE)
    {
        car.forward(SPEED_FULL, 1500);
        delay(5000);
        car.reverse(SPEED_FULL, 1500);
        delay(5000);
        car.turn_clockwise(SPEED_FULL, 1500);
        delay(5000);
        car.turn_counterclockwise(SPEED_FULL, 1500);
        delay(5000);
    }
}

void handle_command ()
{
    if (false)
    {
        if (irrecv.decode(&ir_results))  // have we received an IR signal?
        {
            const unsigned long value = ir_results.value;
            const char cmd = '.'; //translate_ir(value);
            Serial.print(value);
            Serial.print(" IR Command: ");
            Serial.println(cmd);
            switch (cmd)
            {
                case '<':
                    car.reverse(SPEED_FULL, 500);
                    break;
                case '>':
                    car.forward(SPEED_FULL, 500);
                    break;
                case '=':
                    car.all_stop();
                    break;
                case 'd':
                    car.turn_clockwise(SPEED_FULL, 500);
                    break;
                case 'u':
                    car.turn_counterclockwise(SPEED_FULL, 500);
                    break;
            }
            delay(500);
            irrecv.resume();  // receive the next value
        }
    }
    if (Serial.available())
    {
        const char cmd = Serial.read();
        // Discard newlines
        if (cmd == '\n' || cmd == '\r')
            return;
        Serial.print("Command: ");
        Serial.println(cmd);
        switch (cmd)
        {
            case 'b':
                car.reverse(SPEED_FULL, 500);
                break;
            case 'c':
                command_mode_command();
                break;
//            case 'd':
//                demo_mode_command();
//                break;
            case 'd':
                car.demo_drive_leds();
                break;
            case 'f':
                car.forward(SPEED_FULL, 500);
                break;
            case 'l':
                car.turn_clockwise(SPEED_FULL, 500);
                break;
            case 'r':
                car.turn_counterclockwise(SPEED_FULL, 500);
                break;
            case 's':
                car.all_stop();
                break;
            case 'A':  // Red m0
                car.drive_forward(0, SPEED_FULL);
                delay(1000);
                car.drive_stop(0);
                break;
            case 'B':  // Green m0
                car.drive_reverse(0, SPEED_FULL);
                delay(1000);
                car.drive_stop(0);
                break;
            case 'C':  // Red m1
                car.drive_forward(1, SPEED_FULL);
                delay(1000);
                car.drive_stop(1);
                break;
            case 'D':  // Green m1
                car.drive_reverse(1, SPEED_FULL);
                delay(1000);
                car.drive_stop(1);
                break;
            case 'x':  // motor 0 forward
                car.drive_forward(0, SPEED_FULL);
                delay(1000);
                car.all_stop();
                break;
            case 'X':  // motor 0 reverse
                car.drive_reverse(0, SPEED_FULL);
                delay(1000);
                car.all_stop();
                break;
            case 'y':  // motor 1 forward
                car.drive_forward(1, SPEED_FULL);
                delay(1000);
                car.all_stop();
                break;
            case 'Y':  // motor 1 reverse
                car.drive_reverse(1, SPEED_FULL);
                delay(1000);
                car.all_stop();
                break;
            case 'z':  // motors forward
                car.drive_forward(0, SPEED_FULL);
                car.drive_forward(1, SPEED_FULL);
                delay(1000);
                car.all_stop();
                break;
            case 'Z':  // motors reverse
                car.drive_reverse(1, SPEED_FULL);
                car.drive_reverse(1, SPEED_FULL);
                delay(1000);
                car.all_stop();
                break;
            case '?':
                help_command();
                break;
            default:
                Serial.println("Invalid command");
                break;
        }
        while (Serial.available())
        {
            const char cmd = Serial.peek();
            // Discard newlines
            if (cmd == '\n' || cmd == '\r')
                return;
            Serial.read();
        }
    }
}

void command_mode_command ()
{
    car.all_stop();
    mode = COMMAND_MODE;
    Serial.println("Current mode is COMMAND MODE");
}

void demo_mode_command ()
{
    car.all_stop();
    mode = DEMO_MODE;
    Serial.println("Current mode is DEMO MODE");
}

//void motor_0_command ()
//{
//    car.drive_forward(0, SPEED_FULL);
//    delay(5000);
//    car.all_stop();
//}
//
//void motor_1_command ()
//{
//    car.drive_forward(1, SPEED_FULL);
//    delay(5000);
//    car.all_stop();
//}
//
//void motor_2_command ()
//{
//    car.drive_reverse(0, SPEED_FULL);
//    delay(5000);
//    car.all_stop();
//}
//
//void motor_3_command ()
//{
//    car.drive_reverse(1, SPEED_FULL);
//    delay(5000);
//    car.all_stop();
//}

void help_command ()
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
    else
    {
        Serial.println("Current mode is UNKNOWN MODE");
    }
//    read_imu();
//    print_distance();
}

void print_distance ()
{
    long d = sr04.Distance();
    Serial.print("Distance ");
    Serial.print(d);
    Serial.println(" cm");
}

void read_imu ()
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
    Serial.print(" | AcX = ");
    Serial.println(AcX);
    Serial.print(" | AcY = ");
    Serial.println(AcY);
    Serial.print(" | AcZ = ");
    Serial.println(AcZ);
    Serial.print(" | Tmp = ");
    Serial.println(Tmp / 340.00 + 36.53);  //equation for temperature in degrees C from datasheet
    Serial.print(" | GyX = ");
    Serial.println(GyX);
    Serial.print(" | GyY = ");
    Serial.println(GyY);
    Serial.print(" | GyZ = ");
    Serial.println(GyZ);
    Serial.println();
}

//void forward (const int speed, const int duration)
//{
//    Serial.print("forward ");
//    Serial.print(speed);
//    Serial.print(" for ");
//    Serial.print(duration);
//    Serial.println(" ms");
//    for (int i = 0; i < MOTOR_COUNT; i++)
//    {
//        car.drive_forward(i, speed);
//        delay(10);
//    }
//    delay(duration);
//    Serial.println("stop");
//    car.all_stop();
//}
//
//void reverse (const int speed, const int duration)
//{
//    Serial.print("reverse ");
//    Serial.print(speed);
//    Serial.print(" for ");
//    Serial.print(duration);
//    Serial.println(" ms");
//    for (int i = 0; i < MOTOR_COUNT; i++)
//    {
//        car.drive_reverse(i, speed);
//        delay(10);
//    }
//    delay(duration);
//    Serial.println("stop");
//    car.all_stop();
//}
//
//void turn_clockwise (const int speed, const int duration)
//{
//    Serial.print("turn clockwise ");
//    Serial.print(speed);
//    Serial.print(" for ");
//    Serial.print(duration);
//    Serial.println(" ms");
//
//    car.drive_forward(0, SPEED_FULL);
//    car.drive_reverse(1, SPEED_FULL);
//
//    delay(duration);
//    Serial.println("stop");
//    car.all_stop();
//}
//
//void turn_counterclockwise (const int speed, const int duration)
//{
//    Serial.print("turn counterclockwise ");
//    Serial.print(speed);
//    Serial.print(" for ");
//    Serial.print(duration);
//    Serial.println(" ms");
//
//    car.drive_reverse(0, SPEED_FULL);
//    car.drive_forward(1, SPEED_FULL);
//
//    delay(duration);
//    Serial.println("stop");
//    car.all_stop();
//}

//void set_motor_stop (const int motor)
//{
//    car.drive_stop(motor);
//}


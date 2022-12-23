#include "Arduino.h"
#include <SR04.h>
#include "IRremote.h"
#include <MPU6050.h>
#include <Wire.h>
#include "Car.hpp"
#include "Motor.hpp"
#include "Parser.hpp"

// Program for robot car

void setup_imu ();
void handle_command ();
void execute_command (const String input);
int get_words (const String command, String result[], int max_words);
void print_distance ();

void help_command ();
void read_imu ();

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
        delay(1); // allow time to receive
        const int size = 256;
        char buffer[size];
        int i = 0;
        while (i < size)
        {
            if (Serial.available())
            {
                const char cmd = Serial.read();
                if (cmd == '\n' || cmd == '\r')
                {
                    buffer[i] = '\0';
                    String command(buffer);
                    execute_command(command);
                    return;
                }
                else
                {
                    buffer[i] = cmd;
                    i++;
                }
            }
            else
            {
                delay(1); // allow time to receive
            }
        }
    }
}

int get_words (const String command, String result[], int max_words)
{
    int word = 0;
    int start = 0;
    bool in_word = false;
    for (int i = 0; i < command.length(); i++)
    {
        const char chr = command[i];
        if (isWhitespace(chr))
        {
            if (in_word)
            {
                result[word] = command.substring(start, i);
                word++;
                in_word = false;
                if (word >= max_words)
                    return word;
            }
        }
        else if (!in_word)
        {
            in_word = true;
            start = i;
        }
        // else continue this word
    }
    if (in_word)
    {
        result[word] = command.substring(start);
        word++;
    }
    return word;
}

/** Execute a command from a buffer.
 @param command The full command string with no newline.
 */
void execute_command (const String input)
{
    Serial.print("execute_command: ");
    Serial.println(input);
    const int word_limit = 10;
    String words[word_limit];
    int n = get_words(input, words, word_limit);
//    for (int i = 0; i < n; i++)
//    {
//        Serial.print("Word ");
//        Serial.print(i);
//        Serial.print(": ");
//        Serial.println(words[i]);
//    }
    if (n > 0)
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
            car.reverse(speed, duration);
        }
        else if (command == "c")
        {
            mode = COMMAND_MODE;
            Serial.println("Current mode is COMMAND MODE");
        }
        else if (command == "demo")
        {
            mode = DEMO_MODE;
            Serial.println("Current mode is DEMO_MODE MODE");
        }
        else if (command == "led")
        {
            car.demo_drive_leds();
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
            car.forward(speed, duration);
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
            car.turn_clockwise(speed, duration);
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
            car.turn_counterclockwise(speed, duration);
        }
        else if (command == "s")
        {
            car.all_stop();
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
    else
    {
        Serial.println("no command input");
    }
}

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


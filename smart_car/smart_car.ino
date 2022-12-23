#include "Arduino.h"
#include <SR04.h>
#include "IRremote.h"
#include <MPU6050.h>
#include <Wire.h>
#include "Motor.hpp"

// Program for robot car

void setup_imu ();
void demo_drive_leds ();
void handle_command ();
void print_distance ();
void read_imu ();
void command_mode_command ();
void demo_mode_command ();
void stop_command ();
void motor_0_command ();
void motor_1_command ();
void motor_2_command ();
void motor_3_command ();
void help_command ();
void forward (const int speed, const int duration);
void reverse (const int speed, const int duration);
void turn_clockwise (const int speed, const int duration);
void turn_counterclockwise (const int speed, const int duration);
void all_stop ();
void set_motor_forward (const int motor, const int speed);
void set_motor_reverse (const int motor, int speed);
const char translate_ir (const unsigned long value);

// pins
// 2 yellow = in1
// 3 orange = in2
// 4 purple = in3
// 5 blue = in4
// 6 blue = enA
// 7 green = enB

// LED_0 = 22;  // red led
// LED_1 = 24;  // green led
// LED_2 = 26;  // red led
// LED_3 = 28;  // green led
Motor motors[] =
{ Motor(6, 2, 3, 26, 28), Motor(7, 5, 4, 22, 24) };

const int SPEED_FULL = 255;
const int SPEED_Q3 = 196;
const int SPEED_160 = 175;
const int SPEED_HALF = 128;
const int SPEED_Q1 = 64;
const int SPEED_CRAWL = 32;

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
    //Serial.println("IR Receiver Button Decode");
    //irrecv.enableIRIn();  // Start the receiver
    for (int motor = 0; motor < MOTOR_COUNT; motor++)
    {
        motors[motor].setup();
    }
    //setup_imu();
    demo_drive_leds();
    Serial.println("Ready");
}

void demo_drive_leds ()
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
        forward(SPEED_FULL, 1500);
        delay(5000);
        reverse(SPEED_FULL, 1500);
        delay(5000);
//        turn_clockwise(SPEED_FULL, 1500);
//        delay(5000);
//        turn_counterclockwise(SPEED_FULL, 1500);
//        delay(5000);
    }
}

void handle_command ()
{
    if (false)
    {
        if (irrecv.decode(&ir_results))  // have we received an IR signal?
        {
            const unsigned long value = ir_results.value;
            const char cmd = translate_ir(value);
            Serial.print(value);
            Serial.print(" IR Command: ");
            Serial.println(cmd);
            switch (cmd)
            {
                case '<':
                    reverse(SPEED_FULL, 500);
                    all_stop();
                    break;
                case '>':
                    forward(SPEED_FULL, 500);
                    all_stop();
                    break;
                case '=':
                    stop_command();
                    break;
                case 'd':
                    turn_clockwise(SPEED_FULL, 500);
                    all_stop();
                    break;
                case 'u':
                    turn_counterclockwise(SPEED_FULL, 500);
                    all_stop();
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
                reverse(SPEED_FULL, 500);
                all_stop();
                break;
            case 'c':
                command_mode_command();
                break;
//            case 'd':
//                demo_mode_command();
//                break;
            case 'd':
                demo_drive_leds();
                break;
            case 'f':
                forward(SPEED_FULL, 500);
                all_stop();
                break;
            case 'l':
                turn_clockwise(SPEED_FULL, 500);
                all_stop();
                break;
            case 'r':
                turn_counterclockwise(SPEED_FULL, 500);
                all_stop();
                break;
            case 's':
                stop_command();
                break;
            case '0':
                motor_0_command();
                break;
            case '1':
                motor_1_command();
                break;
            case '2':
                motor_2_command();
                break;
            case '3':
                motor_3_command();
                break;
            case 'A':  // Red m0
                motors[0].drive_forward(SPEED_FULL);
                delay(1000);
                motors[0].drive_stop();
                break;
            case 'B':  // Green m0
                motors[0].drive_reverse(SPEED_FULL);
                delay(1000);
                motors[0].drive_stop();
                break;
            case 'C':  // Red m1
                motors[1].drive_forward(SPEED_FULL);
                delay(1000);
                motors[1].drive_stop();
                break;
            case 'D':  // Green m1
                motors[1].drive_reverse(SPEED_FULL);
                delay(1000);
                motors[1].drive_stop();
                break;
            case 'x':  // motor 0 forward
                motors[0].drive_forward(SPEED_FULL);
                delay(1000);
                all_stop();
                break;
            case 'X':  // motor 0 reverse
                motors[0].drive_reverse(SPEED_FULL);
                delay(1000);
                all_stop();
                break;
            case 'y':  // motor 1 forward
                set_motor_forward(1, SPEED_FULL);
                delay(1000);
                all_stop();
                break;
            case 'Y':  // motor 1 reverse
                set_motor_reverse(1, SPEED_FULL);
                delay(1000);
                all_stop();
                break;
            case 'z':  // motors forward
                set_motor_forward(0, SPEED_FULL);
                set_motor_forward(1, SPEED_FULL);
                delay(1000);
                all_stop();
                break;
            case 'Z':  // motors reverse
                set_motor_reverse(0, SPEED_FULL);
                set_motor_reverse(1, SPEED_FULL);
                delay(1000);
                all_stop();
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
    all_stop();
    mode = COMMAND_MODE;
    Serial.println("Current mode is COMMAND MODE");
}

void demo_mode_command ()
{
    all_stop();
    mode = DEMO_MODE;
    Serial.println("Current mode is DEMO MODE");
}



void counterclockwise_command ()
{
}

void stop_command ()
{
    all_stop();
}

void motor_0_command ()
{
    set_motor_forward(0, SPEED_FULL);
    delay(5000);
    all_stop();
}

void motor_1_command ()
{
    set_motor_forward(1, SPEED_FULL);
    delay(5000);
    all_stop();
}

void motor_2_command ()
{
    set_motor_reverse(0, SPEED_FULL);
    delay(5000);
    all_stop();
}

void motor_3_command ()
{
    set_motor_reverse(1, SPEED_FULL);
    delay(5000);
    all_stop();
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

void forward (const int speed, const int duration)
{
    Serial.print("forward ");
    Serial.print(speed);
    Serial.print(" for ");
    Serial.print(duration);
    Serial.println(" ms");
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        motors[i].drive_forward(speed);
        delay(10);
    }
    delay(duration);
    Serial.println("stop");
    all_stop();
}

void reverse (const int speed, const int duration)
{
    Serial.print("reverse ");
    Serial.print(speed);
    Serial.print(" for ");
    Serial.print(duration);
    Serial.println(" ms");
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        motors[i].drive_reverse(speed);
        delay(10);
    }
    delay(duration);
    Serial.println("stop");
    all_stop();
}

void turn_clockwise (const int speed, const int duration)
{
    Serial.print("turn clockwise ");
    Serial.print(speed);
    Serial.print(" for ");
    Serial.print(duration);
    Serial.println(" ms");

    set_motor_forward(0, SPEED_FULL);
    set_motor_reverse(1, SPEED_FULL);

    delay(duration);
    Serial.println("stop");
    all_stop();
}

void turn_counterclockwise (const int speed, const int duration)
{
    Serial.print("turn counterclockwise ");
    Serial.print(speed);
    Serial.print(" for ");
    Serial.print(duration);
    Serial.println(" ms");

    set_motor_reverse(0, SPEED_FULL);
    set_motor_forward(1, SPEED_FULL);

    delay(duration);
    Serial.println("stop");
    all_stop();
}

void set_motor_stop (const int motor)
{
    motors[motor].drive_stop();
}

void all_stop ()
{
    set_motor_stop(0);
    set_motor_stop(1);
}

void set_motor_forward (const int motor, const int speed)
{
    motors[motor].drive_forward(speed);
    delay(10);
}

void set_motor_reverse (const int motor, int speed)
{
    motors[motor].drive_reverse(speed);
    delay(10);
}

const char translate_ir (const unsigned long value)
{
    switch (value)
    {
        case 0xFFA25D:
            Serial.println("POWER");
            return 'P';
        case 0xFFE21D:
            Serial.println("FUNC/STOP");
            return 'S';
        case 0xFF629D:
            Serial.println("VOL+");
            return '+';
        case 0xFF22DD:
            Serial.println("FAST BACK");
            return '<';
        case 0xFF02FD:
            Serial.println("PAUSE");
            return '=';
        case 0xFFC23D:
            Serial.println("FAST FORWARD");
            return '>';
        case 0xFFE01F:
            Serial.println("DOWN");
            return 'd';
        case 0xFFA857:
            Serial.println("VOL-");
            return '-';
        case 0xFF906F:
            Serial.println("UP");
            return 'u';
        case 0xFF9867:
            Serial.println("EQ");
            return 'Q';
        case 0xFFB04F:
            Serial.println("ST/REPT");
            return 'R';
        case 0xFF6897:
            Serial.println("ir 0");
            return '0';
        case 0xFF30CF:
            Serial.println("ir 1");
            return '1';
        case 0xFF18E7:
            Serial.println("ir 2");
            return '2';
        case 0xFF7A85:
            Serial.println("ir 3");
            return '3';
        case 0xFF10EF:
            Serial.println("ir 4");
            return '4';
        case 0xFF38C7:
            Serial.println("ir 5");
            return '5';
        case 0xFF5AA5:
            Serial.println("ir 6");
            return '6';
        case 0xFF42BD:
            Serial.println("ir 7");
            return '7';
        case 0xFF4AB5:
            Serial.println("ir 8");
            return '8';
        case 0xFF52AD:
            Serial.println("ir 9");
            return '9';
        case 0xFFFFFFFF:
            Serial.println(" REPEAT");
            return '*';

        default:
            Serial.println(" other button   ");
            return '?';
    }
}

/*
 class Event {
 public:
 unsigned long m_when;
 bool m_enable;
 struct Event *m_next;

 Event(unsigned long when)
 : m_when(when), m_enable(true), m_next(NULL) {}

 virtual void execute() {}
 };

 class DigitalWriteEvent : public Event {
 public:
 int m_pin;
 int m_value;

 DigitalWriteEvent(unsigned long when, int pin, int value)
 : Event(when), m_pin(pin), m_value(value) {}

 void execute() {
 m_enable = false;
 digitalWrite(m_pin, m_value);
 }
 };

 class AllStopEvent : public Event {
 public:

 AllStopEvent(unsigned long when)
 : Event(when) {}

 void execute() {
 m_enable = false;
 all_stop();
 }
 };

 Event *next_event = NULL;

 void clear_queue() {
 Event *next = NULL;
 Event e = next_event;
 next_event = NULL;
 while (e != NULL) {
 next = e->m_next;
 delete e;
 e = next;
 }
 }

 void process_events(unsigned long duration) {
 unsigned long deadline = millis() + duration;
 while (millis() < deadline) {
 wait_for_event(deadline - millis());
 check_events();
 }
 }

 void wait_for_event(unsigned long pause) {
 if (next_event == NULL) {
 delay(pause);
 } else {
 while (millis() < next_event->m_when) {
 delay(next_event->m_when - millis());
 }
 }
 }

 void live_delay(unsigned long pause) {
 unsigned long deadline = millis() + pause;
 while (millis() < deadline) {
 handle_command();
 delay(10);
 }
 }

 void check_events() {
 Event *event = next_event;
 if (event != NULL) {
 const unsigned long now = millis();
 if (event->m_when <= now) {
 next_event = event->m_next;
 event->execute();
 delete event;
 }
 }
 }

 void queue_event(Event *event) {
 if (next_event == NULL) {
 event->m_next = NULL;
 next_event = event;
 } else {
 for (Event *e = next_event; e != NULL; e = e->m_next) {
 if (e->m_next == NULL || event->m_when < e->m_next->m_when) {
 event->m_next = e->m_next;
 e->m_next = event;
 }
 }
 }
 }

 void queue_all_stop(const unsigned long when) {
 queue_event(new AllStopEvent(when));
 }

 void queue_write_event(unsigned long when, int pin, int value) {
 queue_event(new DigitalWriteEvent(when, pin, value));
 }
 */

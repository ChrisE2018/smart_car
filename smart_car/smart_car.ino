#include "Arduino.h"
#include <SR04.h>
#include "IRremote.h"
#include <MPU6050.h>
#include <Wire.h>

// Early program for robot car

void setup_drive_motors ();
void setup_drive_leds ();
void setup_imu ();
void demo_drive_leds ();
void handle_command ();
void print_distance ();
void read_imu ();
void backward_command ();
void command_mode_command ();
void demo_mode_command ();
void forward_command ();
void left_command ();
void right_command ();
void stop_command ();
void motor_0_command ();
void motor_1_command ();
void help_command ();
void forward (const int speed, const int duration);
void reverse (const int speed, const int duration);
void forward_right (const int speed, const int duration);
void forward_left (const int speed, const int duration);
void all_stop ();
void set_motor_speed (const int motor, const int enable);
void set_motor_forward (const int motor, const int speed);
void set_motor_reverse (const int motor, int speed);
const char translate_ir (const unsigned long value);

int enable_pin[] =
{ 5, 10 };  // Enable motor: chip pins 1, 9
int dir_a_pin[] =
{ 3, 8 };    // Forward if high: chip pins 7, 15
int dir_b_pin[] =
{ 4, 9 };    // reverse if low: chip pins 2, 9

const int SPEED_FULL = 255;
const int SPEED_Q3 = 196;
const int SPEED_HALF = 128;
const int SPEED_Q1 = 64;
const int SPEED_CRAWL = 32;

const int LED_0 = 22;  // red
const int LED_1 = 24;  // green
const int LED_2 = 26;  // red
const int LED_3 = 28;  // orange

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
    Serial.println("IR Receiver Button Decode");
    irrecv.enableIRIn();  // Start the receiver
    setup_drive_motors();
    setup_drive_leds();
    setup_imu();
    demo_drive_leds();
}

void setup_drive_motors ()
{
    for (int motor = 0; motor < 2; motor++)
    {
        //---set pin direction
        pinMode(enable_pin[motor], OUTPUT);
        pinMode(dir_a_pin[motor], OUTPUT);
        pinMode(dir_b_pin[motor], OUTPUT);
    }
}

void setup_drive_leds ()
{
    pinMode(LED_0, OUTPUT);
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);
}

void demo_drive_leds ()
{
    for (int i = 0; i < 3; i++)
    {
        digitalWrite(LED_0, HIGH);
        delay(100);
        digitalWrite(LED_0, LOW);
        delay(100);
        digitalWrite(LED_1, HIGH);
        delay(100);
        digitalWrite(LED_1, LOW);
        delay(100);
        digitalWrite(LED_2, HIGH);
        delay(100);
        digitalWrite(LED_2, LOW);
        delay(100);
        digitalWrite(LED_3, HIGH);
        delay(100);
        digitalWrite(LED_3, LOW);
        delay(100);
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

// TODO Commands from remote
// TODO Handle remote
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
        forward_right(SPEED_FULL, 1500);
        delay(5000);
        forward_left(SPEED_FULL, 1500);
        delay(5000);
    }
}

void handle_command ()
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
                backward_command();
                break;
            case '>':
                forward_command();
                break;
            case '=':
                stop_command();
                break;
            case 'd':
                left_command();
                break;
            case 'u':
                right_command();
                break;
        }
        delay(500);
        irrecv.resume();  // receive the next value
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
                backward_command();
                break;
            case 'c':
                command_mode_command();
                break;
            case 'd':
                demo_mode_command();
                break;
            case 'f':
                forward_command();
                break;
            case 'l':
                left_command();
                break;
            case 'r':
                right_command();
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

void backward_command ()
{
    reverse(SPEED_FULL, 1500);
    delay(5000);
    all_stop();
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

void forward_command ()
{
    forward(SPEED_FULL, 1500);
    delay(5000);
    all_stop();
}

void left_command ()
{
    forward_left(SPEED_FULL, 1500);
    delay(5000);
    all_stop();
}

void right_command ()
{
    forward_right(SPEED_FULL, 1500);
    delay(5000);
    all_stop();
}

void stop_command ()
{
    all_stop();
}

void motor_0_command ()
{
    all_stop();
    set_motor_forward(0, HIGH);
    delay(5000);
    all_stop();
}

void motor_1_command ()
{
    all_stop();
    set_motor_forward(1, HIGH);
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
    read_imu();
    print_distance();
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

void led_off ()
{
    digitalWrite(LED_0, LOW);
    digitalWrite(LED_1, LOW);
    digitalWrite(LED_2, LOW);
    digitalWrite(LED_3, LOW);
}
void led_on ()
{
    digitalWrite(LED_0, HIGH);
    digitalWrite(LED_1, HIGH);
    digitalWrite(LED_2, HIGH);
    digitalWrite(LED_3, HIGH);
}

void led_forward (int motor)
{
    if (motor == 0)
    {
        digitalWrite(LED_0, HIGH);
        digitalWrite(LED_1, LOW);
    }
    else if (motor == 1)
    {
        digitalWrite(LED_2, HIGH);
        digitalWrite(LED_3, LOW);
    }
}

void led_reverse (int motor)
{
    if (motor == 0)
    {
        digitalWrite(LED_0, LOW);
        digitalWrite(LED_1, HIGH);
    }
    else if (motor == 1)
    {
        digitalWrite(LED_2, LOW);
        digitalWrite(LED_3, HIGH);
    }
}

void forward (const int speed, const int duration)
{
    led_off();
    Serial.print("forward ");
    Serial.print(speed);
    Serial.print(" for ");
    Serial.print(duration);
    Serial.println(" ms");
    for (int i = 0; i < 2; i++)
    {
        set_motor_forward(i, HIGH);
    }
    delay(duration);
    Serial.println("stop");
    all_stop();
}

void reverse (const int speed, const int duration)
{
    led_off();
    Serial.print("reverse ");
    Serial.print(speed);
    Serial.print(" for ");
    Serial.print(duration);
    Serial.println(" ms");
    for (int i = 0; i < 2; i++)
    {
        set_motor_reverse(i, HIGH);
    }
    delay(duration);
    Serial.println("stop");
    all_stop();
}

void forward_right (const int speed, const int duration)
{
    led_off();
    digitalWrite(LED_0, LOW);
    digitalWrite(LED_1, LOW);
    Serial.print("forward right ");
    Serial.print(speed);
    Serial.print(" for ");
    Serial.print(duration);
    Serial.println(" ms");

    set_motor_forward(0, HIGH);
    set_motor_reverse(1, HIGH);

    delay(duration);
    Serial.println("stop");
    all_stop();
}

void forward_left (const int speed, const int duration)
{
    led_off();
    digitalWrite(LED_2, LOW);
    digitalWrite(LED_3, LOW);
    Serial.print("forward left ");
    Serial.print(speed);
    Serial.print(" for ");
    Serial.print(duration);
    Serial.println(" ms");

    set_motor_reverse(0, HIGH);
    set_motor_forward(1, HIGH);

    delay(duration);
    Serial.println("stop");
    all_stop();
}

void all_stop ()
{
    set_motor_speed(0, LOW);
    set_motor_speed(1, LOW);
    led_off();
}

void set_motor_speed (const int motor, const int enable)
{
    digitalWrite(enable_pin[motor], enable);
}

void set_motor_forward (const int motor, const int speed)
{
    led_forward(motor);
    digitalWrite(dir_a_pin[motor], HIGH);
    digitalWrite(dir_b_pin[motor], LOW);
    set_motor_speed(motor, speed);
}

void set_motor_reverse (const int motor, int speed)
{
    led_reverse(motor);
    digitalWrite(dir_a_pin[motor], LOW);
    digitalWrite(dir_b_pin[motor], HIGH);
    set_motor_speed(motor, speed);
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

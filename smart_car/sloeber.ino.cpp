#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2022-12-23 13:46:54

#include "Arduino.h"
#include "Arduino.h"
#include <SR04.h>
#include "IRremote.h"
#include <MPU6050.h>
#include <Wire.h>

void setup () ;
void setup_drive_motors () ;
void setup_drive_leds () ;
void demo_drive_leds () ;
void setup_imu () ;
void loop () ;
void handle_command () ;
void backward_command () ;
void command_mode_command () ;
void demo_mode_command () ;
void forward_command () ;
void left_command () ;
void right_command () ;
void stop_command () ;
void motor_0_command () ;
void motor_1_command () ;
void motor_2_command () ;
void motor_3_command () ;
void help_command () ;
void print_distance () ;
void read_imu () ;
void set_all_led (int value) ;
void forward (const int speed, const int duration) ;
void reverse (const int speed, const int duration) ;
void forward_right (const int speed, const int duration) ;
void forward_left (const int speed, const int duration) ;
void set_motor_stop (const int motor) ;
void all_stop () ;
void set_motor_forward (const int motor, const int speed) ;
void set_motor_reverse (const int motor, int speed) ;
const char translate_ir (const unsigned long value) ;

#include "smart_car.ino"


#endif

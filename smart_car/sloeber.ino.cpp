#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2022-12-23 18:10:49

#include "Arduino.h"
#include "Arduino.h"
#include <SR04.h>
#include "IRremote.h"
#include <MPU6050.h>
#include <Wire.h>
#include "Car.hpp"
#include "Motor.hpp"

void setup () ;
void setup_imu () ;
void loop () ;
void handle_command () ;
int get_words (const String command, String result[], int max_words) ;
void execute_command (const String input) ;
void help_command () ;
void print_distance () ;
void read_imu () ;

#include "smart_car.ino"


#endif

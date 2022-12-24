#include "Arduino.h"
#include "IRremote.h"
#include "Car.hpp"
#include "Motor.hpp"

// Program for robot car

Car car;

/*-----Infrared remote-----*/
const int ir_receiver_pin = 30;  // Signal Pin of IR receiver to Arduino Digital Pin 11
IRrecv irrecv(ir_receiver_pin);  // create instance of 'irrecv'
decode_results ir_results;       // create instance of 'decode_results'

/* Control program. */

void setup ()
{
    Serial.begin(9600);
    Serial.println("Smart car");
    car.setup();
    car.demo_drive_leds();
    //Serial.println("IR Receiver Button Decode");
    //irrecv.enableIRIn();  // Start the receiver

    Serial.println("Ready");
}

// TODO Event based

void loop ()
{
    car.cycle();
}


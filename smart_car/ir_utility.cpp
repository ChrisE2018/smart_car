/*
 * ir_utility.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "ir_utility.h"
#include "Arduino.h"
#include "IRremote.h"

/*-----Infrared remote-----*/
const int ir_receiver_pin = 30;  // Signal Pin of IR receiver to Arduino Digital Pin 11
IRrecv irrecv(ir_receiver_pin);  // create instance of 'irrecv'
decode_results ir_results;       // create instance of 'decode_results'

void setup_ir ()
{
    Serial.println("IR Receiver Button Decode");
    irrecv.enableIRIn();  // Start the receiver
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

const char get_ir_command ()
{
    if (irrecv.decode(&ir_results))  // have we received an IR signal?
    {
        const unsigned long value = ir_results.value;
        const char cmd = translate_ir(value);
        Serial.print(value);
        Serial.print(" IR Command: ");
        Serial.println(cmd);
        irrecv.resume();  // receive the next value
        return cmd;
    }
    return '.';
}


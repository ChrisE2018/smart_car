/*
 * Heap.cpp
 *
 *  Created on: Jan 10, 2023
 *      Author: cre
 */

#include <Arduino.h>
#include "Heap.hpp"

// For some reason Serial does not always resolve.
extern HardwareSerial Serial;

static const unsigned long initial_stack_address = SP;

void print_heap_state ()
{
    unsigned long heap_address = get_heap_address();
    unsigned long heap_size = RAMEND - heap_address;
    long stack_size = initial_stack_address - SP;
    unsigned long free_ram = get_free_ram();
    Serial.print(F("Heap Usage: "));
    Serial.print(heap_size);
    Serial.print(F(" / "));
    Serial.print(heap_size + free_ram);
    Serial.print(F(" Stack Size: "));
    Serial.print(stack_size);
    Serial.print(F(" Free RAM: "));
    Serial.print(free_ram);
    Serial.print(F(" [Stack: "));
    Serial.print(initial_stack_address);
    Serial.print(F(":"));
    Serial.print(SP);
    Serial.print(F(" Heap: "));
    Serial.print(RAMEND);
    Serial.print(F(":"));
    Serial.print(heap_address);
    Serial.println(F("]"));
}

unsigned long get_free_ram ()
{
    int *heap_var = (int*) malloc(sizeof(int));
    unsigned long heap_var_address = (unsigned long) heap_var;
    free(heap_var);

    return SP - heap_var_address;
}

unsigned long get_heap_address ()
{
    int *heap_var = (int*) calloc(1, sizeof(int));
    unsigned long heap_address = (unsigned long) heap_var;
    free(heap_var);
    return heap_address;
}


/*
 * Heap.cpp
 *
 *  Created on: Jan 10, 2023
 *      Author: cre
 */

#include <Arduino.h>
#include "Heap.hpp"
#include "../logging/Logger.hpp"

static Logger logger(__FILE__, Level::info);

static const unsigned long initial_stack_address = SP;

void print_heap_state ()
{
    const unsigned long heap_address = get_heap_address();
    const unsigned long heap_size = RAMEND - heap_address;
    const long stack_size = initial_stack_address - SP;
    const unsigned long free_ram = get_free_ram();
    const unsigned long sp = SP;
    const unsigned long ramend = RAMEND;
    LOG_INFO(logger, "Heap Usage: %lu / %lu Stack Size: %lu Free RAM: %lu [Stack %lu:%lu Heap %lu:%lu]", heap_size,
            heap_size + free_ram, stack_size, free_ram, initial_stack_address, sp, ramend, heap_address);
}

void get_heap_state (char *buffer, size_t size)
{
    const unsigned long heap_address = get_heap_address();
    const unsigned long heap_size = RAMEND - heap_address;
    const long stack_size = initial_stack_address - SP;
    const unsigned long free_ram = get_free_ram();
    const unsigned long sp = SP;
    const unsigned long ramend = RAMEND;
    snprintf(buffer, size, "Heap Usage: %lu / %lu Stack Size: %lu Free RAM: %lu [Stack %lu:%lu Heap %lu:%lu]",
            heap_size, heap_size + free_ram, stack_size, free_ram, initial_stack_address, sp, ramend, heap_address);
}

unsigned long get_free_ram ()
{
    char *const heap_var = (char*) malloc(sizeof(char));
    const unsigned long heap_address = (unsigned long) heap_var;
    free(heap_var);

    return SP - heap_address;
}

unsigned long get_heap_address ()
{
    char *const heap_var = (char*) malloc(sizeof(char));
    const unsigned long heap_address = (unsigned long) heap_var;
    free(heap_var);
    return heap_address;
}


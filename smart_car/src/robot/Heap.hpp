/*
 * Heap.hpp
 *
 *  Created on: Jan 10, 2023
 *      Author: cre
 *
 * Based on: https://forum.arduino.cc/t/getting-heap-size-stack-size-and-free-ram-from-due/678195
 */

#pragma once

void print_heap_state ();
void get_heap_state (char *buffer, size_t size);
unsigned long get_free_ram ();
unsigned long get_heap_address ();

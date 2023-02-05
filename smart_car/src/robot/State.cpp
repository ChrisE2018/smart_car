/*
 * Copyright (c) 2023 by Christopher Eliot.
 *
 * State.cpp
 *
 *  Created on: Feb 2, 2023
 *      Author: cre
 */

#include "State.hpp"

State::State (State &source)
{
    values[0] = source.values[0];
    values[1] = source.values[1];
    values[2] = source.values[2];
    values[3] = source.values[3];
    values[4] = source.values[4];
    values[5] = source.values[5];
    values[6] = source.values[6];
    values[7] = source.values[7];
    values[8] = source.values[8];
}

State::State (float _values[])
{
    values[0] = _values[0];
    values[1] = _values[1];
    values[2] = _values[2];
    values[3] = _values[3];
    values[4] = _values[4];
    values[5] = _values[5];
    values[6] = _values[6];
    values[7] = _values[7];
    values[8] = _values[8];
}

State::State (float x, float y, float angle, float vx, float vy, float vangle, float ax, float ay, float aangle)
{
    values[0] = x;
    values[1] = y;
    values[2] = angle;
    values[3] = vx;
    values[4] = vy;
    values[5] = vangle;
    values[6] = ax;
    values[7] = ay;
    values[8] = aangle;
}

float State::get_x () const
{
    return values[0];
}

float State::get_y () const
{
    return values[1];
}

float State::get_angle () const
{
    return values[2];
}

float State::get_vx () const
{
    return values[3];
}

float State::get_vy () const
{
    return values[4];
}

float State::get_vangle () const
{
    return values[5];
}

float State::get_ax () const
{
    return values[6];
}

float State::get_ay () const
{
    return values[7];
}

float State::get_aangle () const
{
    return values[8];
}

void State::set (float x = 0, float y = 0, float angle = 0, float vx = 0, float vy = 0, float vangle = 0, float ax = 0,
        float ay = 0, float aangle = 0)
{
    values[0] = x;
    values[1] = y;
    values[2] = angle;
    values[3] = vx;
    values[4] = vy;
    values[5] = vangle;
    values[6] = ax;
    values[7] = ay;
    values[8] = aangle;
}

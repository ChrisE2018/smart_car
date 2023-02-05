/*
 * Copyright (c) 2023 by Christopher Eliot.
 *
 * State.hpp
 *
 *  Created on: Feb 2, 2023
 *      Author: cre
 */

#pragma once

class State
{
    public:
        static constexpr int SIZE = 9;
        State (State &source);
        State (float values[]);
        State (float x = 0, float y = 0, float angle = 0, float vx = 0, float vy = 0, float vangle = 0, float ax = 0,
                float ay = 0, float aangle = 0);
        float get_x() const;
        float get_y() const;
        float get_angle() const;
        float get_vx() const;
        float get_vy() const;
        float get_vangle() const;
        float get_ax() const;
        float get_ay() const;
        float get_aangle() const;

        void set(float x = 0, float y = 0, float angle = 0, float vx = 0, float vy = 0, float vangle = 0, float ax = 0,
                float ay = 0, float aangle = 0);

        float values[SIZE];
};


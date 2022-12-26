/*
 * Cyclic.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include "Mode.hpp"

class Cyclic
{
    public:
        Cyclic ();
        virtual ~Cyclic ();

        Mode get_mode ();
        virtual void set_mode (Mode mode);
        virtual void cycle ();
    private:
        Mode mode = COMMAND_MODE;
};


/*
 * RobotAppender.hpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#pragma once

#include "Appender.hpp"

class RobotAppender : public Appender
{
    public:
        RobotAppender ();
        virtual ~RobotAppender () = default;
        virtual void append (const Logger* logger, const Level level, const int line,
                const char *message);
    private:
        const int buffer_size = 256;
};


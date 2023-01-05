/*
 * RobotAppender.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#include "RobotAppender.hpp"

RobotAppender::RobotAppender ()
{
}

void RobotAppender::append (const Logger *logger, const Level level, const int line,
        const char *message)
{
    char buf[buffer_size];
    snprintf(buf, buffer_size, "%s [%s] %s", logger->get_short_name().c_str(), stringify(level), message);
    Serial.println(buf);
    Serial1.println(buf);
}

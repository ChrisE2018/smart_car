/*
 * RobotAppender.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#include "RobotAppender.hpp"
#include "../plugins/ClockPlugin.hpp"

RobotAppender::RobotAppender (Car &car) : car(car)
{
}

void RobotAppender::append (const Logger *logger, const Level level, const int line,
        const char *message)
{
    time_t t = 0;
    ClockPlugin *clock_plugin = car.get_clock_plugin();
    if (clock_plugin != nullptr)
    {
        t = clock_plugin->get_unixtime();
    }
    struct tm *lt = localtime(&t);
    char buf[buffer_size];
    snprintf(buf, buffer_size, "%s [%s %s] %s", isotime(lt), logger->get_short_name().c_str(),
            stringify(level), message);
    Serial.println(buf);
    Serial1.println(buf);
}

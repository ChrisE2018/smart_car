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
    ClockPlugin *const clock_plugin = car.get_clock_plugin();
    if (clock_plugin != nullptr)
    {
        t = clock_plugin->get_unixtime();
    }
    struct tm *lt = localtime(&t);
    const int ms = millis() % 1000;
    char buf[buffer_size];
    snprintf(buf, buffer_size, "%s.%03d [%s %s:%d] %s", isotime(lt), ms, stringify(level),
            logger->get_short_name().c_str(), line, message);
    Serial.println(buf);
    Serial3.println(buf);
    // [TODO] Also append to log file
}

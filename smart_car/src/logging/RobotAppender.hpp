/*
 * RobotAppender.hpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#pragma once

#include "Appender.hpp"
#include "../robot/car.hpp"
#include <SD.h>
class Car;

class RobotAppender : public Appender
{
    public:
        RobotAppender (Car &car);
        virtual ~RobotAppender () = default;
        virtual void append (const Logger *logger, const Level level, const int line,
                const char *message);
        void open_logfile();
        void flush();
        void close();
    private:
        const int buffer_size = 256;
        Car &car;
        // For file access on micro SD card
        const int chipSelect = 53;
        File log_file;
        String get_logfile();
};


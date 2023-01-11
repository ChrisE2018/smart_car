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
        virtual void append (const Logger *const logger, const Level level, const int line,
                const char *const message);
        void append (const char *const message);
        void open_logfile ();
        void flush ();
        void close ();
    private:
        Car &car;

        // For file access on micro SD card
        const int chipSelect = 53;
        static const int filename_size = 16;
        char log_filename[filename_size];
        File log_file;
        static const int buffer_size = 256;
        char buffer[buffer_size];

        void get_logfile ();
};


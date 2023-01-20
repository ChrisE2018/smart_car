/*
 * RobotAppender.hpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#pragma once

#include "Appender.hpp"
#include "TimeSource.hpp"
#include <SD.h>

namespace logging
{

class Car;

#define LOG_DATA(fmt, args...) robot_appender->log_data_p((const char *)F(fmt), args);
#define FLUSH_DATA(fmt, args...) robot_appender->flush();

class RobotAppender: public Appender
{
    public:
        RobotAppender (const Level level, Formatter &formatter, TimeSource &time_source);
        virtual ~RobotAppender () = default;
        void append (const Level level, const char *const message);
        void append_usb (const char *const message);
        void append_bluetooth (const char *const message);
        void append_file (const char *const message);
        void open_logfile ();
        void flush ();
        void close ();
        void enable_usb_logger (const bool enable);
        void enable_bluetooth_logger (const bool enable);
        void enable_file_logger (const bool enable);
        bool is_usb_logger ();
        bool is_bluetooth_logger ();
        bool is_file_logger ();
        bool get_logger_state (const String &mode) const;
        void set_logger_state (const String &mode, const String &state);
        bool log_data (String folder, String filename, const char *message);
        bool save_data (String folder, String filename, const char *message);

    private:
        TimeSource &time_source;

        // For file access on micro SD card
        const int chipSelect = 53;
        static const int filename_size = 32;
        char log_filename[filename_size];
        File log_file;

        bool usb_logger = true;
        bool bluetooth_logger = true;
        bool file_logger = true;

        void set_log_pathname ();
};

}

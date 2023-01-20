/*
 * SdAppender.hpp
 *
 *  Created on: Jan 20, 2023
 *      Author: cre
 */

#pragma once

#include "Appender.hpp"
#include "TimeSource.hpp"
#include <SD.h>

namespace logging
{

class SdAppender: public Appender
{
    public:
        SdAppender (const Level level, Formatter &formatter, TimeSource &time_source);
        virtual ~SdAppender () = default;
        virtual void append (const Level level, const char *const message);
        void open_logfile ();
        void flush ();
        void close ();
        bool log_data (String folder, String filename, const char *message);
        bool save_data (String folder, String filename, const char *message);

    private:
        TimeSource &time_source;

        // For file access on micro SD card
        const int chipSelect = 53;
        static const int filename_size = 32;
        char log_filename[filename_size];
        File log_file;

        void set_log_pathname ();
};

}

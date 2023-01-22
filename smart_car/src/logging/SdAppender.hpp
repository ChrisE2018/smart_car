/*
 * Copyright 2023 Christopher Eliot
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
        SdAppender (const int chip_select, const Level level, Formatter &formatter, TimeSource &time_source);
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
        int chip_select;
        static const int filename_size = 32;
        char log_filename[filename_size];
        File log_file;

        void set_log_pathname ();
};

}

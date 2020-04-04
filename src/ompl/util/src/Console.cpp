/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include "ompl/util/Console.h"
#include <mutex>
#include <iostream>
#include <cstdio>
#include <cstdarg>
#ifdef _WIN32
#include <stdio.h>
#include <io.h>
#define isatty(x) _isatty(x)
#define fileno(x) _fileno(x)
#else
#include <unistd.h>
#endif

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

/// @cond IGNORE

struct DefaultOutputHandler
{
    DefaultOutputHandler()
    {
        output_handler_ = static_cast<ompl::msg::OutputHandler *>(&std_output_handler_);
        previous_output_handler_ = output_handler_;
        logLevel_ = ompl::msg::LOG_DEBUG;
    }

    ompl::msg::OutputHandlerSTD std_output_handler_;
    ompl::msg::OutputHandler *output_handler_;
    ompl::msg::OutputHandler *previous_output_handler_;
    ompl::msg::LogLevel logLevel_;
    std::mutex lock_;  // it is likely the outputhandler does some I/O, so we serialize it
};

// we use this function because we want to handle static initialization correctly
// however, the first run of this function is not thread safe, due to the use of a static
// variable inside the function. For this reason, we ensure the first call happens during
// static initialization using a proxy class
static DefaultOutputHandler *getDOH()
{
    static DefaultOutputHandler DOH;
    return &DOH;
}

#define USE_DOH                                                                                                        \
    DefaultOutputHandler *doh = getDOH();                                                                              \
    std::lock_guard<std::mutex> slock(doh->lock_)

#define MAX_BUFFER_SIZE 1024

/// @endcond

void ompl::msg::noOutputHandler()
{
    USE_DOH;
    doh->previous_output_handler_ = doh->output_handler_;
    doh->output_handler_ = nullptr;
}

void ompl::msg::restorePreviousOutputHandler()
{
    USE_DOH;
    std::swap(doh->previous_output_handler_, doh->output_handler_);
}

void ompl::msg::useOutputHandler(OutputHandler *oh)
{
    USE_DOH;
    doh->previous_output_handler_ = doh->output_handler_;
    doh->output_handler_ = oh;
}

ompl::msg::OutputHandler *ompl::msg::getOutputHandler()
{
    return getDOH()->output_handler_;
}

void ompl::msg::log(const char *file, int line, LogLevel level, const char *m, ...)
{
    USE_DOH;
    if ((doh->output_handler_ != nullptr) && level >= doh->logLevel_)
    {
        va_list __ap;
        va_start(__ap, m);
        char buf[MAX_BUFFER_SIZE];
        vsnprintf(buf, sizeof(buf), m, __ap);
        va_end(__ap);
        buf[MAX_BUFFER_SIZE - 1] = '\0';

        doh->output_handler_->log(buf, level, file, line);
    }
}

void ompl::msg::setLogLevel(LogLevel level)
{
    USE_DOH;
    doh->logLevel_ = level;
}

ompl::msg::LogLevel ompl::msg::getLogLevel()
{
    USE_DOH;
    return doh->logLevel_;
}

static const char *LogLevelString[6] = {"Dev2:    ", "Dev1:    ", "Debug:   ", "Info:    ", "Warning: ", "Error:   "};
static const char *LogColorString[6] = {ANSI_COLOR_MAGENTA, ANSI_COLOR_GREEN,  ANSI_COLOR_BLUE,
                                        ANSI_COLOR_CYAN,    ANSI_COLOR_YELLOW, ANSI_COLOR_RED};

void ompl::msg::OutputHandlerSTD::log(const std::string &text, LogLevel level, const char *filename, int line)
{
    if (level >= LOG_WARN)
    {
        bool isTTY(isatty(fileno(stderr)) != 0);
        if (isTTY)
            std::cerr << LogColorString[level + 2];
        std::cerr << LogLevelString[level + 2] << text << std::endl;
        std::cerr << "         at line " << line << " in " << filename << std::endl;
        if (isTTY)
            std::cerr << ANSI_COLOR_RESET;
        std::cerr.flush();
    }
    else
    {
        bool isTTY(isatty(fileno(stdout)) != 0);
        if (isTTY)
            std::cout << LogColorString[level + 2];
        std::cout << LogLevelString[level + 2] << text << std::endl;
        if (isTTY)
            std::cout << ANSI_COLOR_RESET;
        std::cout.flush();
    }
}

ompl::msg::OutputHandlerFile::OutputHandlerFile(const char *filename)
{
    file_ = fopen(filename, "a");
    if (file_ == nullptr)
        std::cerr << "Unable to open log file: '" << filename << "'" << std::endl;
}

ompl::msg::OutputHandlerFile::~OutputHandlerFile()
{
    if (file_ != nullptr)
        if (fclose(file_) != 0)
            std::cerr << "Error closing logfile" << std::endl;
}

void ompl::msg::OutputHandlerFile::log(const std::string &text, LogLevel level, const char *filename, int line)
{
    if (file_ != nullptr)
    {
        fprintf(file_, "%s%s\n", LogLevelString[level + 2], text.c_str());
        if (level >= LOG_WARN)
            fprintf(file_, "         at line %d in %s\n", line, filename);
        fflush(file_);
    }
}

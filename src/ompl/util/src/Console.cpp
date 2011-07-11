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
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>

static ompl::msg::OutputHandlerSTD DEFAULT_OUTPUT_HANDLER;
static ompl::msg::OutputHandler   *OUTPUT_HANDLER = static_cast<ompl::msg::OutputHandler*>(&DEFAULT_OUTPUT_HANDLER);
static ompl::msg::OutputHandler   *PREVIOUS_OH = OUTPUT_HANDLER;
static boost::mutex                LOCK; // it is likely the outputhandler does some I/O, so we serialize it

void ompl::msg::noOutputHandler(void)
{
    LOCK.lock();
    PREVIOUS_OH = OUTPUT_HANDLER;
    OUTPUT_HANDLER = NULL;
    LOCK.unlock();
}

void ompl::msg::restorePreviousOutputHandler(void)
{
    LOCK.lock();
    std::swap(PREVIOUS_OH, OUTPUT_HANDLER);
    LOCK.unlock();
}

void ompl::msg::useOutputHandler(OutputHandler *oh)
{
    LOCK.lock();
    PREVIOUS_OH = OUTPUT_HANDLER;
    OUTPUT_HANDLER = oh;
    LOCK.unlock();
}

ompl::msg::OutputHandler* ompl::msg::getOutputHandler(void)
{
    return OUTPUT_HANDLER;
}

ompl::msg::Interface::Interface(const std::string &prefix)
{
    setPrefix(prefix);
}

ompl::msg::Interface::~Interface(void)
{
}

const std::string& ompl::msg::Interface::getPrefix(void) const
{
    return prefix_;
}

void ompl::msg::Interface::setPrefix(const std::string &prefix)
{
    prefix_ = prefix;
    if (!prefix_.empty())
        prefix_ += ": ";
}

// the maximum buffer size to use when printing a message
#define MAX_BUFFER_SIZE 1024

// macro that combines the set of arguments into a single string
#define COMBINE(m, buf, size)                                                \
    va_list __ap;                                                        \
    va_start(__ap, m);                                                        \
    char buf##_chr[size];                                                \
    vsnprintf(buf##_chr, sizeof(buf##_chr), m, __ap);                        \
    va_end(__ap);                                                        \
    buf##_chr[size - 1] = '\0';                                                \
    std::string buf(buf##_chr)

void ompl::msg::Interface::debug(const std::string &text) const
{
    if (OUTPUT_HANDLER)
    {
        LOCK.lock();
        OUTPUT_HANDLER->debug(prefix_ + text);
        LOCK.unlock();
    }
}

void ompl::msg::Interface::debug(const char *msg, ...) const
{
    COMBINE(msg, buf, MAX_BUFFER_SIZE);
    debug(buf);
}

void ompl::msg::Interface::inform(const std::string &text) const
{
    if (OUTPUT_HANDLER)
    {
        LOCK.lock();
        OUTPUT_HANDLER->inform(prefix_ + text);
        LOCK.unlock();
    }
}

void ompl::msg::Interface::inform(const char *msg, ...) const
{
    COMBINE(msg, buf, MAX_BUFFER_SIZE);
    inform(buf);
}

void ompl::msg::Interface::warn(const std::string &text) const
{
    if (OUTPUT_HANDLER)
    {
        LOCK.lock();
        OUTPUT_HANDLER->warn(prefix_ + text);
        LOCK.unlock();
    }
}

void ompl::msg::Interface::warn(const char *msg, ...) const
{
    COMBINE(msg, buf, MAX_BUFFER_SIZE);
    warn(buf);
}

void ompl::msg::Interface::error(const std::string &text) const
{
    if (OUTPUT_HANDLER)
    {
        LOCK.lock();
        OUTPUT_HANDLER->error(prefix_ + text);
        LOCK.unlock();
    }
}

void ompl::msg::Interface::error(const char *msg, ...) const
{
    COMBINE(msg, buf, MAX_BUFFER_SIZE);
    error(buf);
}

void ompl::msg::OutputHandlerSTD::error(const std::string &text)
{
    std::cerr << "Error:   " << text << std::endl;
    std::cerr.flush();
}

void ompl::msg::OutputHandlerSTD::warn(const std::string &text)
{
    std::cerr << "Warning: " << text << std::endl;
    std::cerr.flush();
}

void ompl::msg::OutputHandlerSTD::inform(const std::string &text)
{
    std::cout << "Info:    " << text << std::endl;
    std::cout.flush();
}

void ompl::msg::OutputHandlerSTD::debug(const std::string &text)
{
    std::cout << "Debug:   " << text << std::endl;
    std::cout.flush();
}

ompl::msg::OutputHandlerFile::OutputHandlerFile(const char *filename) : OutputHandler()
{
    file_ = fopen(filename, "a");
    if (!file_)
        std::cerr << "Unable to open log file: '" << filename << "'" << std::endl;
}

ompl::msg::OutputHandlerFile::~OutputHandlerFile(void)
{
    if (file_)
        if (fclose(file_) != 0)
            std::cerr << "Error closing logfile" << std::endl;
}

void ompl::msg::OutputHandlerFile::error(const std::string &text)
{
    if (file_)
    {
        fprintf(file_, "Error:   %s\n", text.c_str());
        fflush(file_);
    }
}

void ompl::msg::OutputHandlerFile::warn(const std::string &text)
{
    if (file_)
    {
        fprintf(file_, "Warning: %s\n", text.c_str());
        fflush(file_);
    }
}

void ompl::msg::OutputHandlerFile::inform(const std::string &text)
{
    if (file_)
    {
        fprintf(file_, "Info:    %s\n", text.c_str());
        fflush(file_);
    }
}

void ompl::msg::OutputHandlerFile::debug(const std::string &text)
{
    if (file_)
    {
        fprintf(file_, "Debug:   %s\n", text.c_str());
        fflush(file_);
    }
}

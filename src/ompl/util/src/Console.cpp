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

/// @cond IGNORE

// If we are using OMPL in conjunction with other libraries, it is possible those libs end up calling functions
// for configuring the console output before constructors of static vars are called for OMPL. This is why we do
// not rely on the constructors for statics being called here. Instead, we use the getDOH() routine,
// which allocates the objects when needed.

struct DefaultOutputHandler
{
    DefaultOutputHandler(void)
    {
        output_handler_ = static_cast<ompl::msg::OutputHandler*>(&std_output_handler_);
        previous_output_handler_ = output_handler_;
    }

    ompl::msg::OutputHandlerSTD std_output_handler_;
    ompl::msg::OutputHandler   *output_handler_;
    ompl::msg::OutputHandler   *previous_output_handler_;
    boost::mutex                lock_; // it is likely the outputhandler does some I/O, so we serialize it
};

static DefaultOutputHandler *DOH = NULL;

// automatically destruct the static instance above
struct DefaultOutputHandlerDestructor
{
    ~DefaultOutputHandlerDestructor(void)
    {
        if (DOH)
        {
            delete DOH;
            DOH = NULL;
        }
    }
};

static DefaultOutputHandlerDestructor DOHD;

static inline DefaultOutputHandler* getDOH(void)
{
    if (DOH)
        return DOH;
    else
    {
        static boost::mutex alloc;
        boost::mutex::scoped_lock slock(alloc);
        if (DOH == NULL)
            DOH = new DefaultOutputHandler();
    }
    return DOH;
}

#define USE_DOH                                                                \
    DefaultOutputHandler *doh = getDOH();                                \
    boost::mutex::scoped_lock slock(doh->lock_)

/// @endcond

void ompl::msg::noOutputHandler(void)
{
    USE_DOH;
    doh->previous_output_handler_ = doh->output_handler_;
    doh->output_handler_ = NULL;
}

void ompl::msg::restorePreviousOutputHandler(void)
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

ompl::msg::OutputHandler* ompl::msg::getOutputHandler(void)
{
    return getDOH()->output_handler_;
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

/// @cond IGNORE

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

#define CALL_DOH(fn, arg)                                                \
    USE_DOH;                                                                \
    if (doh->output_handler_)                                                \
        doh->output_handler_->fn(arg)

/// @endcond

void ompl::msg::Interface::debug(const std::string &text) const
{
    CALL_DOH(debug, prefix_ + text);
}

void ompl::msg::Interface::debug(const char *msg, ...) const
{
    COMBINE(msg, buf, MAX_BUFFER_SIZE);
    debug(buf);
}

void ompl::msg::Interface::inform(const std::string &text) const
{
    CALL_DOH(inform, prefix_ + text);
}

void ompl::msg::Interface::inform(const char *msg, ...) const
{
    COMBINE(msg, buf, MAX_BUFFER_SIZE);
    inform(buf);
}

void ompl::msg::Interface::warn(const std::string &text) const
{
    CALL_DOH(warn, prefix_ + text);
}

void ompl::msg::Interface::warn(const char *msg, ...) const
{
    COMBINE(msg, buf, MAX_BUFFER_SIZE);
    warn(buf);
}

void ompl::msg::Interface::error(const std::string &text) const
{
    CALL_DOH(error, prefix_ + text);
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

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

#ifndef OMPL_UTIL_CONSOLE_
#define OMPL_UTIL_CONSOLE_

#include <string>

/** \file Console.h
    \defgroup logging Logging Macros
    \{

    \def OMPL_ERROR(fmt, ...)
    \brief Log a formatted error string.
    \remarks This macro takes the same arguments as [printf](http://www.cplusplus.com/reference/clibrary/cstdio/printf).

    \def OMPL_WARN(fmt, ...)
    \brief Log a formatted warning string.
    \remarks This macro takes the same arguments as [printf](http://www.cplusplus.com/reference/clibrary/cstdio/printf).

    \def OMPL_INFORM(fmt, ...)
    \brief Log a formatted information string.
    \remarks This macro takes the same arguments as [printf](http://www.cplusplus.com/reference/clibrary/cstdio/printf).

    \def OMPL_DEBUG(fmt, ...)
    \brief Log a formatted debugging string.
    \remarks This macro takes the same arguments as [printf](http://www.cplusplus.com/reference/clibrary/cstdio/printf).

    \}
*/
#define OMPL_ERROR(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_ERROR, fmt, ##__VA_ARGS__)

#define OMPL_WARN(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_WARN, fmt, ##__VA_ARGS__)

#define OMPL_INFORM(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_INFO, fmt, ##__VA_ARGS__)

#define OMPL_DEBUG(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_DEBUG, fmt, ##__VA_ARGS__)

#define OMPL_DEVMSG1(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_DEV1, fmt, ##__VA_ARGS__)

#define OMPL_DEVMSG2(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_DEV2, fmt, ##__VA_ARGS__)

namespace ompl
{
    /** \brief Message namespace. This contains classes needed to
        output error messages (or logging) from within the library.
        Message logging can be performed with \ref logging "logging macros" */
    namespace msg
    {
        /** \brief The set of priorities for message logging */
        enum LogLevel
        {
            LOG_DEV2 = -2,  // message type for developers
            LOG_DEV1 = -1,  // message type for developers
            LOG_DEBUG = 0,
            LOG_INFO,
            LOG_WARN,
            LOG_ERROR,
            LOG_NONE
        };

        /** \brief Generic class to handle output from a piece of
            code.

            In order to handle output from the library in different
            ways, an implementation of this class needs to be
            provided. This instance can be set with the useOutputHandler
            function. */
        class OutputHandler
        {
        public:
            OutputHandler() = default;

            virtual ~OutputHandler() = default;

            /** \brief log a message to the output handler with the given text
                and logging level from a specific file and line number */
            virtual void log(const std::string &text, LogLevel level, const char *filename, int line) = 0;
        };

        /** \brief Default implementation of OutputHandler. This sends
            the information to the console. */
        class OutputHandlerSTD : public OutputHandler
        {
        public:
            OutputHandlerSTD() = default;

            void log(const std::string &text, LogLevel level, const char *filename, int line) override;
        };

        /** \brief Implementation of OutputHandler that saves messages in a file. */
        class OutputHandlerFile : public OutputHandler
        {
        public:
            /** \brief The name of the file in which to save the message data */
            OutputHandlerFile(const char *filename);

            ~OutputHandlerFile() override;

            void log(const std::string &text, LogLevel level, const char *filename, int line) override;

        private:
            /** \brief The file to save to */
            FILE *file_;
        };

        /** \brief This function instructs ompl that no messages should be outputted. Equivalent to
         * useOutputHandler(nullptr) */
        void noOutputHandler();

        /** \brief Restore the output handler that was previously in use (if any) */
        void restorePreviousOutputHandler();

        /** \brief Specify the instance of the OutputHandler to use. By default, this is OutputHandlerSTD */
        void useOutputHandler(OutputHandler *oh);

        /** \brief Get the instance of the OutputHandler currently used. This is nullptr in case there is no output
         * handler. */
        OutputHandler *getOutputHandler();

        /** \brief Set the minimum level of logging data to output.  Messages
            with lower logging levels will not be recorded. */
        void setLogLevel(LogLevel level);

        /** \brief Retrieve the current level of logging data.  Messages
            with lower logging levels will not be recorded. */
        LogLevel getLogLevel();

        /** \brief Root level logging function.  This should not be invoked directly,
            but rather used via a \ref logging "logging macro".  Formats the message
            string given the arguments and forwards the string to the output handler */
        void log(const char *file, int line, LogLevel level, const char *m, ...);
    }
}

#endif

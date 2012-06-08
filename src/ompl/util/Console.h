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

    \def logError(fmt, ...)
    \brief Log a formatted error string.
    \remarks This macro takes the same arguments as <a href="http://www.cplusplus.com/reference/clibrary/cstdio/printf">printf</a>.

    \def logWarn(fmt, ...)
    \brief Log a formatted warning string.
    \remarks This macro takes the same arguments as <a href="http://www.cplusplus.com/reference/clibrary/cstdio/printf">printf</a>.

    \def logInform(fmt, ...)
    \brief Log a formatted information string.
    \remarks This macro takes the same arguments as <a href="http://www.cplusplus.com/reference/clibrary/cstdio/printf">printf</a>.

    \def logDebug(fmt, ...)
    \brief Log a formatted debugging string.
    \remarks This macro takes the same arguments as <a href="http://www.cplusplus.com/reference/clibrary/cstdio/printf">printf</a>.

    \}
*/
#define logError(fmt, ...)  ompl::msg::log(__FILE__, __LINE__, ompl::msg::ERROR, fmt, ##__VA_ARGS__)

#define logWarn(fmt, ...)   ompl::msg::log(__FILE__, __LINE__, ompl::msg::WARN,  fmt, ##__VA_ARGS__)

#define logInform(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::INFO,  fmt, ##__VA_ARGS__)

#define logDebug(fmt, ...)  ompl::msg::log(__FILE__, __LINE__, ompl::msg::DEBUG, fmt, ##__VA_ARGS__)

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
            DEBUG = 0,
            INFO,
            WARN,
            ERROR,
            NONE
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

            OutputHandler(void)
            {
            }

            virtual ~OutputHandler(void)
            {
            }

            /** \brief Print an error message: "Error: ...." */
            virtual void error(const std::string &text) = 0;

            /** \brief Print an warning message: "Warning: ...." */
            virtual void warn(const std::string &text) = 0;

            /** \brief Print some information: "Information: ...." */
            virtual void inform(const std::string &text) = 0;

            /** \brief Print a debug message */
            virtual void debug(const std::string &text) = 0;
        };

        /** \brief Default implementation of OutputHandler. This sends
            the information to the console. */
        class OutputHandlerSTD : public OutputHandler
        {
        public:

            OutputHandlerSTD(void) : OutputHandler()
            {
            }

            virtual void error(const std::string &text);

            virtual void warn(const std::string &text);

            virtual void inform(const std::string &text);

            virtual void debug(const std::string &text);

        };

        /** \brief Implementation of OutputHandler that saves messages in a file. */
        class OutputHandlerFile : public OutputHandler
        {
        public:

            /** \brief The name of the file in which to save the message data */
            OutputHandlerFile(const char *filename);

            virtual ~OutputHandlerFile(void);

            virtual void error(const std::string &text);

            virtual void warn(const std::string &text);

            virtual void inform(const std::string &text);

            virtual void debug(const std::string &text);

        private:

            /** \brief The file to save to */
            FILE *file_;

        };

        /** \brief This function instructs ompl that no messages should be outputted. Equivalent to useOutputHandler(NULL) */
        void noOutputHandler(void);

        /** \brief Restore the output handler that was previously in use (if any) */
        void restorePreviousOutputHandler(void);

        /** \brief Specify the instance of the OutputHandler to use. By default, this is OutputHandlerSTD */
        void useOutputHandler(OutputHandler *oh);

        /** \brief Get the instance of the OutputHandler currently used. This is NULL in case there is no output handler. */
        OutputHandler* getOutputHandler(void);

        /** \brief Set the minimum level of logging data to output.  Messages
            with lower logging levels will not be recorded. */
        void setLogLevel(LogLevel level);

        /** \brief Retrieve the current level of logging data.  Messages
            with lower logging levels will not be recorded. */
        LogLevel getLogLevel(void);

        /** \brief Root level logging function.  This should not be invoked directly,
            but rather used via a \ref logging "logging macro".  Formats the message
            string given the arguments and forwards the string to the output handler */
        void log(const char *file, int line, LogLevel level, const char* m, ...);

        /** \brief Toggle output of line number and file name in logging */
        void showLineNumbers(bool show);
    }

}

#endif

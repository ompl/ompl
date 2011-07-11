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
#include <cstdarg>

namespace ompl
{

    /** \brief Message namespace. This contains classes needed to
        output error messages (or logging) from within the library. */
    namespace msg
    {

        /** \brief The piece of code that desires interaction with an
            action or an output handler should use an instance of this
            class. This class connects to the active OutputHandler (if
            any) and forwards messages. */
        class Interface
        {
        public:

            /** \brief The text that will appear in front of every
                message forwarded by this Interface instance can be
                optionally set as a constructor argument. */
            explicit
            Interface(const std::string &prefix = "");
            virtual ~Interface(void);

            /** \brief Set the text that will appear in front of every
                message forwarded by this Interface instance. */
            void setPrefix(const std::string &prefix);

            /** \brief Get the text that appears in front of every forwarded message. */
            const std::string& getPrefix(void) const;

            /** \brief Forward information */
            void inform(const std::string &text) const;

            /** \brief Forward a warning */
            void warn(const std::string &text) const;

            /** \brief Forward an error */
            void error(const std::string &text) const;

            /** \brief Forward a debug message */
            void debug(const std::string &text) const;

            /** \brief Forward information */
            void inform(const char *msg, ...) const;

            /** \brief Forward a warning */
            void warn(const char *msg, ...) const;

            /** \brief Forward an error */
            void error(const char *msg, ...) const;

            /** \brief Forward a debug message */
            void debug(const char *msg, ...) const;

        protected:

            /** \brief The string that appears in front of very forwarded message. */
            std::string prefix_;
        };

        /** \brief Generic class to handle output from a piece of
            code.

            In order to handle output from the library in different
            ways, an implementation of this class needs to be
            provided. The Interface class forwards calls to an
            instance of OutputHandler. This instance can be set with
            the useOutputHandler function. */
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

        /** \brief This function instructs ompl that no messages should be outputted. Equivalent to setOutputHandler(NULL) */
        void noOutputHandler(void);

        /** \brief Restore the output handler that was previously in use (if any) */
        void restorePreviousOutputHandler(void);

        /** \brief Specify the instance of the OutputHandler to use. By default, this is OutputHandlerSTD */
        void useOutputHandler(OutputHandler *oh);

        /** \brief Get the instance of the OutputHandler currently used. This is NULL in case there is no output handler. */
        OutputHandler* getOutputHandler(void);
    }

}

#endif

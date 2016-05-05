/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

#ifndef OMPL_UTIL_TIME_
#define OMPL_UTIL_TIME_

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

// std::put_time is not implemented in GCC 4.x
#if defined(__GNUC__) && (__GNUC__ < 5)
namespace std
{
    inline std::string put_time(const std::tm *tmb, const char *fmt)
    {
        char mbstr[100];
        std::strftime(mbstr, sizeof(mbstr), fmt, tmb);
        return string(mbstr);
    }
}
#endif

namespace ompl
{
    /** \brief Namespace containing time datatypes and time operations */
    namespace time
    {
        /** \brief Representation of a point in time */
        typedef std::chrono::system_clock::time_point point;

        /** \brief Representation of a time duration */
        typedef std::chrono::system_clock::duration duration;

        /** \brief Get the current time point */
        inline point now()
        {
            return std::chrono::system_clock::now();
        }

        /** \brief Return the time duration representing a given number of seconds */
        inline duration seconds(double sec)
        {
            long s = (long)sec;
            long us = (long)((sec - (double)s) * 1000000);
            return std::chrono::seconds(s) + std::chrono::microseconds(us);
        }

        /** \brief Return the number of seconds that a time duration represents */
        inline double seconds(const duration &d)
        {
            return std::chrono::duration<double>(d).count();
        }

        /** \brief Return string representation of point in time */
        inline std::string as_string(const point &p)
        {
            std::time_t pt = std::chrono::system_clock::to_time_t(p);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&pt), "%F %T");
            return ss.str();
        }
    }
}

#endif

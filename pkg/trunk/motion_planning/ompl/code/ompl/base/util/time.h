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

/* Taken from ROS implementation */
#ifndef OMPL_BASE_UTIL_TIME_
#define OMPL_BASE_UTIL_TIME_

/* This file is taken from ROS, adaptations by Ioan Sucan */

#include <cmath>
#include <stdint.h>

#ifdef _WIN32
#include <windows.h>
#endif

namespace ompl
{

    namespace time_utils
    {
	
	class Duration
	{
	public:
	    int32_t sec, nsec;
	Duration() : sec(0), nsec(0) { }
	    Duration(int32_t _sec, int32_t _nsec);
	    Duration(double d);
	    Duration(int64_t t);
	    Duration operator+(const Duration &rhs) const;
	    Duration operator-(const Duration &rhs) const;
	    bool operator==(const Duration &rhs) const;
	    bool operator>(const Duration &rhs) const;
	    bool operator<(const Duration &rhs) const;
	    double toSeconds() const { return (double)sec + 1e-9*(double)nsec; }
	    int64_t toNSeconds() const { return (int64_t)sec * 1000000000 + (int64_t)nsec;  }
	    /** sleeps for the time duration specified by this object */
	    bool sleep(void) const;
	};
	
	class Time
	{
	public:
	    uint32_t sec, nsec;
#ifdef _WIN32
	    // unless I've missed something obvious, the only way to get high-precision
	    // time on Windows is via the QueryPerformanceCounter() call. However,
	    // this is somewhat problematic in Windows XP on some processors, especially 
	    // AMD, because the Windows implementation can freak out when the CPU clocks
	    // down to save power. Time can jump or even go backwards. Microsoft has
	    // fixed this bug for most systems now, but it can still show up if you have
	    // not installed the latest CPU drivers (an oxymoron). They fixed all these
	    // problems in Windows Vista, and this API is by far the most accurate that
	    // I know of in Windows, so I'll use it here despite all these caveats
	    static LARGE_INTEGER cpu_freq, init_cpu_time;
	    static Time start_time;
#endif
	Time() : sec(0), nsec(0) { }
	    Time(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec) { }
	    Time(uint64_t t) : sec(t / 1000000000), nsec(t % 1000000000) { }
	    Duration operator-(const Time &rhs) const;
	    Time operator+(const Duration &rhs) const;
	    bool operator==(const Time &rhs) const;
	    bool operator>(const Time &rhs) const;
	    bool operator<(const Time &rhs) const;
	    static Time now();
	    double toSeconds() const { return (double)sec + 1e-9*(double)nsec; }
	    uint64_t toNSeconds() const { return (uint64_t)sec*1000000000 + (uint64_t)nsec;  }
	    bool isZero() const { return sec == 0 && nsec == 0; }
	};	
    }
}

#endif
    

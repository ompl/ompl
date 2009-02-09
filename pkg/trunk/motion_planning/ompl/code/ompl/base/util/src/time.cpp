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

/* This file is taken from ROS, adaptations by Ioan Sucan */

#include "ompl/base/util/time.h"
#include <cmath>
#include <time.h>

#ifndef _WIN32
  #if _POSIX_TIMERS <= 0
  #include <sys/time.h>
  #endif
#else
  #include <sys/timeb.h>
  static ompl::Time ompl::Time::start_time;
#endif
  
using namespace ompl::time_utils;
using namespace std;

Duration::Duration(int32_t _sec, int32_t _nsec)
  : sec(_sec), nsec(_nsec)
{
    while (nsec > 1000000000)
    {
	nsec -= 1000000000;
	sec++;
    }
    while (nsec < 0)
    {
	nsec += 1000000000;
	sec--;
    }
}

Duration::Duration(double d)
{
    if (d > 0)
	sec = (int32_t)floor(d);
    else
	sec = (int32_t)floor(d) + 1;
    
    nsec = (int32_t)((d - (double)sec)*1000000000);
}

Duration::Duration(int64_t t)
{
    sec  = (int32_t)(t / 1000000000);
    nsec = (int32_t)(t % 1000000000);
    while (nsec > 1000000000)
    {
	nsec -= 1000000000;
	sec++;
    }
    while (nsec < 0)
    {
	nsec += 1000000000;
	sec--;
    }
}

Duration Time::operator-(const Time &rhs) const
{
    return Duration((int32_t)sec -  (int32_t)rhs.sec, 
		    (int32_t)nsec - (int32_t)rhs.nsec); // carry handled in ctor
}

Time Time::operator+(const Duration &rhs) const
{
    int64_t sec_sum  = (int64_t)sec  + (int64_t)rhs.sec;
    int64_t nsec_sum = (int64_t)nsec + (int64_t)rhs.nsec;
    while (nsec_sum < 0)
    {
	nsec_sum += 1000000000;
	sec_sum--;
    }
    while (nsec_sum > 1000000000)
    {
	nsec_sum -= 1000000000;
	sec_sum++;
    }
    return Time((uint32_t)sec_sum, (uint32_t)nsec_sum);
}

Duration Duration::operator+(const Duration &rhs) const
{
    return Duration(sec + rhs.sec, nsec + rhs.nsec);
}

Duration Duration::operator-(const Duration &rhs) const
{
    return Duration(sec - rhs.sec, nsec - rhs.nsec);
}

bool Duration::operator<(const Duration &rhs) const
{
    if (sec < rhs.sec)
	return true;
    else if (sec == rhs.sec && nsec < rhs.nsec)
	return true;
    return false;
}

bool Duration::operator>(const Duration &rhs) const
{
    if (sec > rhs.sec)
	return true;
    else if (sec == rhs.sec && nsec > rhs.nsec)
	return true;
    return false;
}

Time Time::now(void)
{
    Time t;
#ifndef _WIN32
#if _POSIX_TIMERS > 0
    struct timespec start;
    clock_gettime(CLOCK_REALTIME, &start);
    t.sec  = start.tv_sec;
    t.nsec = start.tv_nsec;
#else
    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    t.sec  = timeofday.tv_sec;
    t.nsec = timeofday.tv_usec * 1000;
#endif
    return t;
#else
    if (start_time.isZero())
    {
	QueryPerformanceFrequency(&cpu_freq);
	if (cpu_freq.QuadPart == 0)
	{
	    printf("woah! this system (for whatever reason) does not support the "
		   "high-performance timing API. ur done.\n");
	    abort();
	}
	QueryPerformanceCounter(&init_cpu_time);
	// compute an offset from the Epoch using the lower-performance timer API
	FILETIME ft;
	GetSystemTimeAsFileTime(&ft);
	LARGE_INTEGER start_li;
	start_li.LowPart = ft.dwLowDateTime;
	start_li.HighPart = ft.dwHighDateTime;
	// why did they choose 1601 as the time zero, instead of 1970?
	// there were no outstanding hard rock bands in 1601.
#ifdef _MSC_VER
	start_li.QuadPart -= 116444736000000000Ui64;
#else
	start_li.QuadPart -= 116444736000000000ULL;
#endif
	start_time.sec = (uint32_t)(start_li.QuadPart / 10000000); // 100-ns units. odd.
	start_time.nsec = (start_li.LowPart % 10000000) * 100;
    }
    LARGE_INTEGER cur_time;
    QueryPerformanceCounter(&cur_time);
    LARGE_INTEGER delta_cpu_time;
    delta_cpu_time.QuadPart = cur_time.QuadPart - init_cpu_time.QuadPart;
    // todo: how to handle cpu clock drift. not sure it's a big deal for us.
    // also, think about clock wraparound. seems extremely unlikey, but possible
    double d_delta_cpu_time = delta_cpu_time.QuadPart / (double)cpu_freq.QuadPart;
    return Time(start_time + Duration(d_delta_cpu_time));
#endif
}

bool Duration::operator==(const Duration &rhs) const
{
    return sec == rhs.sec && nsec == rhs.nsec;
}

bool Time::operator==(const Time &rhs) const
{
  return sec == rhs.sec && nsec == rhs.nsec;
}

bool Time::operator<(const Time &rhs) const
{
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec < rhs.nsec)
    return true;
  return false;
}

bool Time::operator>(const Time &rhs) const
{
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec > rhs.nsec)
    return true;
  return false;
}

bool Duration::sleep(void) const
{
    struct timespec ts = {sec, nsec};
    return nanosleep(&ts, NULL) ? false : true;
}

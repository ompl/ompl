#include "ompl/base/util/time.h"
#include <cmath>
#include <time.h>
#include <iomanip>
#include <stdexcept>

#ifndef _WIN32
  #if _POSIX_TIMERS <= 0
  #include <sys/time.h>
  #endif
#else
  #include <sys/timeb.h>
  ompl::Time ompl::Time::start_time;
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

Duration::Duration(long long t)
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

Duration Time::operator-(const Time &rhs)
{
  return Duration((int32_t)sec -  (int32_t)rhs.sec, 
                  (int32_t)nsec - (int32_t)rhs.nsec); // carry handled in ctor
}

Time Time::operator+(const Duration &rhs)
{
  long long sec_sum  = (long long)sec  + (long long)rhs.sec;
  long long nsec_sum = (long long)nsec + (long long)rhs.nsec;
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
  if (sec_sum < 0 || sec_sum > 4294967295U)
    throw std::runtime_error("time + duration was out of dual 32-bit range");
  // now, it's safe to downcast back to uint32 bits
  return Time((uint32_t)sec_sum, (uint32_t)nsec_sum);
}

Duration Duration::operator+(const Duration &rhs)
{
  return Duration(sec + rhs.sec, nsec + rhs.nsec);
}

Duration Duration::operator-(const Duration &rhs)
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
  if (start_time.is_zero())
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

bool Duration::sleep(void)
{
  struct timespec ts = {sec, nsec};
  return nanosleep(&ts, NULL) ? false : true;
}

ostream &ompl::operator<<(ostream &os, const time_utils::Time &rhs)
{
  os << rhs.sec << "." << setw(9) << setfill('0') << rhs.nsec;
  return os;
}

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


/* Author Ioan Sucan */

#ifndef OMPL_UTIL_PROFILER_
#define OMPL_UTIL_PROFILER_

#define ENABLE_PROFILING 1

#ifndef ENABLE_PROFILING

/** The ENABLE_PROFILING macro can be set externally. If it is not,
    profiling is enabled by default, unless NDEBUG is defined. */

#  ifdef NDEBUG
#    define ENABLE_PROFILING 0
#  else
#    define ENABLE_PROFILING 1
#  endif

#endif

#if ENABLE_PROFILING

#include <map>
#include <string>
#include <iostream>
#include <boost/thread.hpp>

#include "ompl/util/Time.h"

namespace ompl
{

    /** \brief This is a simple thread-safe tool for counting time
	spent in various chunks of code. This is different from
	external profiling tools in that it allows the user to count
	time spent in various bits of code (sub-function granularity)
	or count how many times certain pieces of code are executed.*/

    class Profiler
    {
    public:
	
	/** \brief Return an instance of the class */
	static Profiler* Instance(void);
	
	/** \brief Constructor. It is allowed to separately instantiate this
	    class (not only as a singleton) */
	Profiler(bool printOnDestroy = false) : running_(false), printOnDestroy_(printOnDestroy)
	{
	}
	
	/** \brief Destructor */
	~Profiler(void)
	{
	    if (printOnDestroy_ && !data_.empty())
		status();
	}
	
	/** \brief Start counting time */
	static void Start(void)
	{
	    Instance()->start();
	}

	/** \brief Stop counting time */	
	static void Stop(void)
	{
	    Instance()->stop();
	}

	/** \brief Clear counted time and events */
	static void Clear(void)
	{
	    Instance()->clear();
	}
	
	/** \brief Start counting time */
	void start(void);

	/** \brief Stop counting time */	
	void stop(void);
	
	/** \brief Clear counted time and events */
	void clear(void);
	
	/** \brief Count a specific event for a number of times */
	static void Event(const std::string& name, const unsigned int times = 1)
	{
	    Instance()->event(name, times);
	}
	
	/** \brief Count a specific event for a number of times */
	void event(const std::string &name, const unsigned int times = 1);
	
	/** \brief Begin counting time for a specific chunk of code */
	static void Begin(const std::string &name)
	{
	    Instance()->begin(name);
	}
	
	/** \brief Stop counting time for a specific chunk of code */
	static void End(const std::string &name)
	{
	    Instance()->end(name);
	}

	/** \brief Begin counting time for a specific chunk of code */
	void begin(const std::string &name);
	
	/** \brief Stop counting time for a specific chunk of code */
	void end(const std::string &name);	
	
	/** \brief Print the status of the profiled code chunks and
	    events. Optionally, computation done by different threads
	    can be printed separately. */
	static void Status(std::ostream &out = std::cout, bool merge = true)
	{
	    Instance()->status(out, merge);
	}
	
	/** \brief Print the status of the profiled code chunks and
	    events. Optionally, computation done by different threads
	    can be printed separately. */
	void status(std::ostream &out = std::cout, bool merge = true);
	
    private:
	
	struct TimeInfo
	{
	    time::duration total;
	    time::point    start;
	    
	    void set(void)
	    {
		start = time::now();
	    }
	    
	    void update(void)
	    {
		total = total + (time::now() - start);
	    }
	};
	
	struct PerThread
	{
	    std::map<std::string, unsigned long int> events;
	    std::map<std::string, TimeInfo>          time;
	};
	
	void printThreadInfo(std::ostream &out, const PerThread &data) const;	
	    
	std::map<boost::thread::id, PerThread> data_;
	TimeInfo                               tinfo_;
	bool                                   running_;
	bool                                   printOnDestroy_;
	
    };
    
}

#else

#include <string>
#include <iostream>

/* If profiling is disabled, provide empty implementations for the
   public functions */
namespace ompl
{

    class Profiler
    {
    public:
	
	static Profiler* Instance(void)
	{
	    return NULL;
	}
	
	Profiler(void)
	{
	}
	
	~Profiler(void)
	{
	}
	
	static void Start(void)
	{
	}

	static void Stop(void)
	{
	}
	
	static void Clear(void)
	{
	}
	
	void start(void)
	{
	}
	
	void stop(void)
	{
	}
	
	void clear(void)
	{
	}
	
	static void Event(const std::string&, const unsigned int = 1)
	{
	}
	
	void event(const std::string &, const unsigned int = 1)
	{
	}
	
	static void Begin(const std::string &)
	{
	}
	
	static void End(const std::string &)
	{
	}

	void begin(const std::string &)
	{
	}

	void end(const std::string &)
	{
	}
	
	static void Status(std::ostream & = std::cout, bool = true)
	{
	}
	
	void status(std::ostream & = std::cout, bool = true)
	{
	}
    };
}

#endif


#endif

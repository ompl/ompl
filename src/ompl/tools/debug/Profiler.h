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

#ifndef OMPL_TOOLS_DEBUG_PROFILER_
#define OMPL_TOOLS_DEBUG_PROFILER_

#ifndef ENABLE_PROFILING

/** The ENABLE_PROFILING macro can be set externally. If it is not,
    profiling is enabled by default, unless NDEBUG is defined. */

#ifdef NDEBUG
#define ENABLE_PROFILING 0
#else
#define ENABLE_PROFILING 1
#endif

#endif

#if ENABLE_PROFILING

#include <map>
#include <string>
#include <iostream>
#include <thread>
#include <mutex>

#include "ompl/util/Time.h"

namespace ompl
{
    namespace tools
    {
        /** \brief This is a simple thread-safe tool for counting time
            spent in various chunks of code. This is different from
            external profiling tools in that it allows the user to count
            time spent in various bits of code (sub-function granularity)
            or count how many times certain pieces of code are executed.*/
        class Profiler
        {
        public:
            // non-copyable
            Profiler(const Profiler &) = delete;
            Profiler &operator=(const Profiler &) = delete;

            /** \brief This instance will call Profiler::begin() when constructed and Profiler::end() when it goes out
             * of scope. */
            class ScopedBlock
            {
            public:
                /** \brief Start counting time for the block named \e name of the profiler \e prof */
                ScopedBlock(const std::string &name, Profiler &prof = Profiler::Instance()) : name_(name), prof_(prof)
                {
                    prof_.begin(name);
                }

                ~ScopedBlock()
                {
                    prof_.end(name_);
                }

            private:
                std::string name_;
                Profiler &prof_;
            };

            /** \brief This instance will call Profiler::start() when constructed and Profiler::stop() when it goes out
               of scope.
                If the profiler was already started, this block's constructor and destructor take no action */
            class ScopedStart
            {
            public:
                /** \brief Take as argument the profiler instance to operate on (\e prof) */
                ScopedStart(Profiler &prof = Profiler::Instance()) : prof_(prof), wasRunning_(prof_.running())
                {
                    if (!wasRunning_)
                        prof_.start();
                }

                ~ScopedStart()
                {
                    if (!wasRunning_)
                        prof_.stop();
                }

            private:
                Profiler &prof_;
                bool wasRunning_;
            };

            /** \brief Return an instance of the class */
            static Profiler &Instance();

            /** \brief Constructor. It is allowed to separately instantiate this
                class (not only as a singleton) */
            Profiler(bool printOnDestroy = false, bool autoStart = false)
              : running_(false), printOnDestroy_(printOnDestroy)
            {
                if (autoStart)
                    start();
            }

            /** \brief Destructor */
            ~Profiler()
            {
                if (printOnDestroy_ && !data_.empty())
                    status();
            }

            /** \brief Start counting time */
            static void Start()
            {
                Instance().start();
            }

            /** \brief Stop counting time */
            static void Stop()
            {
                Instance().stop();
            }

            /** \brief Clear counted time and events */
            static void Clear()
            {
                Instance().clear();
            }

            /** \brief Start counting time */
            void start();

            /** \brief Stop counting time */
            void stop();

            /** \brief Clear counted time and events */
            void clear();

            /** \brief Count a specific event for a number of times */
            static void Event(const std::string &name, const unsigned int times = 1)
            {
                Instance().event(name, times);
            }

            /** \brief Count a specific event for a number of times */
            void event(const std::string &name, const unsigned int times = 1);

            /** \brief Maintain the average of a specific value */
            static void Average(const std::string &name, const double value)
            {
                Instance().average(name, value);
            }

            /** \brief Maintain the average of a specific value */
            void average(const std::string &name, const double value);

            /** \brief Begin counting time for a specific chunk of code */
            static void Begin(const std::string &name)
            {
                Instance().begin(name);
            }

            /** \brief Stop counting time for a specific chunk of code */
            static void End(const std::string &name)
            {
                Instance().end(name);
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
                Instance().status(out, merge);
            }

            /** \brief Print the status of the profiled code chunks and
                events. Optionally, computation done by different threads
                can be printed separately. */
            void status(std::ostream &out = std::cout, bool merge = true);

            /** \brief Print the status of the profiled code chunks and
                events to the console (using msg::Console) */
            static void Console()
            {
                Instance().console();
            }

            /** \brief Print the status of the profiled code chunks and
                events to the console (using msg::Console) */
            void console();

            /** \brief Check if the profiler is counting time or not */
            bool running() const
            {
                return running_;
            }

            /** \brief Check if the profiler is counting time or not */
            static bool Running()
            {
                return Instance().running();
            }

        private:
            /** \brief Information about time spent in a section of the code */
            struct TimeInfo
            {
                TimeInfo()
                  : total(time::seconds(0.)), shortest(time::duration::max()), longest(time::duration::min()), parts(0)
                {
                }

                /** \brief Total time counted. */
                time::duration total;

                /** \brief The shortest counted time interval */
                time::duration shortest;

                /** \brief The longest counted time interval */
                time::duration longest;

                /** \brief Number of times a chunk of time was added to this structure */
                unsigned long int parts;

                /** \brief The point in time when counting time started */
                time::point start;

                /** \brief Begin counting time */
                void set()
                {
                    start = time::now();
                }

                /** \brief Add the counted time to the total time */
                void update()
                {
                    const time::duration &dt = time::now() - start;
                    if (dt > longest)
                        longest = dt;
                    if (dt < shortest)
                        shortest = dt;
                    total = total + dt;
                    ++parts;
                }
            };

            /** \brief Information maintained about averaged values */
            struct AvgInfo
            {
                /** \brief The sum of the values to average */
                double total;

                /** \brief The sub of squares of the values to average */
                double totalSqr;

                /** \brief Number of times a value was added to this structure */
                unsigned long int parts;
            };

            /** \brief Information to be maintained for each thread */
            struct PerThread
            {
                /** \brief The stored events */
                std::map<std::string, unsigned long int> events;

                /** \brief The stored averages */
                std::map<std::string, AvgInfo> avg;

                /** \brief The amount of time spent in various places */
                std::map<std::string, TimeInfo> time;
            };

            void printThreadInfo(std::ostream &out, const PerThread &data);

            std::mutex lock_;
            std::map<std::thread::id, PerThread> data_;
            TimeInfo tinfo_;
            bool running_;
            bool printOnDestroy_;
        };
    }
}

#else

#include <string>
#include <iostream>

/* If profiling is disabled, provide empty implementations for the
   public functions */
namespace ompl
{
    namespace tools
    {
        class Profiler
        {
        public:
            class ScopedBlock
            {
            public:
                ScopedBlock(const std::string &, Profiler & = Profiler::Instance())
                {
                }

                ~ScopedBlock() = default;
            };

            class ScopedStart
            {
            public:
                ScopedStart(Profiler & = Profiler::Instance())
                {
                }

                ~ScopedStart() = default;
            };

            static Profiler &Instance();

            Profiler(bool = true, bool = true)
            {
            }

            ~Profiler() = default;

            static void Start()
            {
            }

            static void Stop()
            {
            }

            static void Clear()
            {
            }

            void start()
            {
            }

            void stop()
            {
            }

            void clear()
            {
            }

            static void Event(const std::string &, const unsigned int = 1)
            {
            }

            void event(const std::string &, const unsigned int = 1)
            {
            }

            static void Average(const std::string &, const double)
            {
            }

            void average(const std::string &, const double)
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

            static void Console()
            {
            }

            void console()
            {
            }

            bool running() const
            {
                return false;
            }

            static bool Running()
            {
                return false;
            }
        };
    }
}

#endif

#endif

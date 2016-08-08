/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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
*   * Neither the name of Rice University nor the names of its
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

#ifndef OMPL_TOOLS_DEBUG_PLANNER_MONITOR_
#define OMPL_TOOLS_DEBUG_PLANNER_MONITOR_

#include <iostream>
#include <boost/scoped_ptr.hpp>
#include <thread>
#include <utility>

#include "ompl/base/Planner.h"

namespace ompl
{
    namespace tools
    {
        /** \brief Monitor the properties a planner exposes, as the planner is running.
            Dump the planner properties to a stream, periodically. */
        class PlannerMonitor
        {
        public:
            // non-copyright
            PlannerMonitor(const PlannerMonitor &) = delete;
            PlannerMonitor &operator=(const PlannerMonitor &) = delete;

            /** \brief Monitor a planner instance, and dump its properties to a specified stream, periodically.

                Every time the properties are dumped, the stream offset is set to 0. It is often useful to have the
                stream be a file, and then issue commands such as 'watch cat filename'. */
            PlannerMonitor(base::PlannerPtr planner, std::ostream &out, double period = 0.5, bool autoStart = true)
              : planner_(std::move(planner)), out_(out), period_(period), shouldMonitor_(false)
            {
                if (autoStart)
                    startMonitor();
            }

            /** \brief Destructor */
            ~PlannerMonitor()
            {
                stopMonitor();
            }

            /** \brief Start the monitoring thread. */
            void startMonitor();

            /** \brief Stop the monitoring thread (automatically stopped by the destructor). */
            void stopMonitor();

        private:
            void threadFunction();

            base::PlannerPtr planner_;
            std::ostream &out_;
            double period_;
            bool shouldMonitor_;
            boost::scoped_ptr<std::thread> monitorThread_;
        };
    }
}

#endif

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

#include "ompl/tools/debug/PlannerMonitor.h"
#include "ompl/util/Time.h"
#include <limits>
#include <functional>

void ompl::tools::PlannerMonitor::startMonitor()
{
    if (monitorThread_)
        return;
    shouldMonitor_ = true;
    monitorThread_.reset(new std::thread([this]
                                         {
                                             threadFunction();
                                         }));
}

void ompl::tools::PlannerMonitor::stopMonitor()
{
    if (!monitorThread_)
        return;
    shouldMonitor_ = false;
    monitorThread_->join();
    monitorThread_.reset();
}

void ompl::tools::PlannerMonitor::threadFunction()
{
    time::point startTime = time::now();
    time::point lastOutputTime = startTime;

    while (shouldMonitor_)
    {
        double timeSinceOutput = time::seconds(time::now() - lastOutputTime);
        if (timeSinceOutput < period_)
        {
            std::this_thread::sleep_for(time::seconds(0.01));
            continue;
        }
        out_.seekp(0);
        out_ << "[T = " << static_cast<unsigned int>(time::seconds(time::now() - startTime) + 0.5) << " s]" << std::endl
             << std::endl;
        out_ << "Planner " << planner_->getName() << ":" << std::endl;
        if (!planner_->isSetup())
        {
            out_ << "Not yet set up." << std::endl;
            return;
        }
        const base::Planner::PlannerProgressProperties &props = planner_->getPlannerProgressProperties();
        for (const auto &prop : props)
        {
            out_ << "    \t * " << prop.first << " \t : " << prop.second() << std::endl;
        }
        out_ << std::endl;
        out_.flush();
        lastOutputTime = time::now();
        std::this_thread::sleep_for(time::seconds(0.01));
    }
}

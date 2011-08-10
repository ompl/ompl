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

#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/util/Time.h"
#include <boost/bind.hpp>
#include <boost/lambda/bind.hpp>
#include <utility>

void ompl::base::PlannerTerminationCondition::terminate(bool flag)
{
    terminate_ = flag;
}

bool ompl::base::PlannerTerminationCondition::eval(void) const
{
    return fn_();
}

ompl::base::PlannerNonTerminatingCondition::PlannerNonTerminatingCondition(void) : PlannerTerminationCondition(boost::lambda::constant(false))
{
}

ompl::base::PlannerAlwaysTerminatingCondition::PlannerAlwaysTerminatingCondition(void) : PlannerTerminationCondition(boost::lambda::constant(true))
{
}

bool ompl::base::PlannerThreadedTerminationCondition::eval(void) const
{
    return evalValue_;
}

void ompl::base::PlannerThreadedTerminationCondition::terminate(bool flag)
{
    PlannerTerminationCondition::terminate(flag);
    if (terminate_)
        stopEvalThread();
    else
        startEvalThread();
}

void ompl::base::PlannerThreadedTerminationCondition::startEvalThread(void)
{
    if (!thread_)
        thread_ = new boost::thread(boost::bind(&PlannerThreadedTerminationCondition::periodicEval, this));
}

void ompl::base::PlannerThreadedTerminationCondition::stopEvalThread(void)
{
    if (thread_)
    {
        thread_->interrupt();
        thread_->join();
        delete thread_;
        thread_ = NULL;
    }
}

bool ompl::base::PlannerThreadedTerminationCondition::computeEval(void)
{
    return fn_();
}

ompl::base::PlannerThreadedTerminationCondition::PlannerThreadedTerminationCondition(double period) :
    PlannerTerminationCondition(), thread_(NULL), evalValue_(terminate_), period_(period)
{
    startEvalThread(); // this is a bit risky, but it seems to work
}

ompl::base::PlannerThreadedTerminationCondition::PlannerThreadedTerminationCondition(const PlannerTerminationConditionFn &fn, double period) :
    PlannerTerminationCondition(fn), thread_(NULL), evalValue_(terminate_), period_(period)
{
    startEvalThread(); // this is a bit risky, but it seems to work
}

ompl::base::PlannerThreadedTerminationCondition::~PlannerThreadedTerminationCondition(void)
{
    terminate_ = true;
    stopEvalThread();
}

void ompl::base::PlannerThreadedTerminationCondition::periodicEval(void)
{
    time::duration s = time::seconds(period_);
    do
    {
        evalValue_ = computeEval();
        if ((*this)())
            break;
        boost::this_thread::sleep(s);
    } while (!(*this)());
}

bool ompl::base::PlannerAndTerminationCondition::eval(void) const
{
    return c1_() && c2_();
}

bool ompl::base::PlannerOrTerminationCondition::eval(void) const
{
    return c1_() || c2_();
}

ompl::base::PlannerTerminationCondition ompl::base::operator*(const PlannerTerminationCondition &a, const PlannerTerminationCondition &b)
{
    return PlannerAndTerminationCondition(a, b);
}

ompl::base::PlannerTerminationCondition ompl::base::operator+(const PlannerTerminationCondition &a, const PlannerTerminationCondition &b)
{
    return PlannerOrTerminationCondition(a, b);
}

/// @cond IGNORE
namespace ompl
{
    // return true if a certain point in time has passed
    static bool timePassed(const time::point &endTime)
    {
        return time::now() > endTime;
    }
}
/// @endcond

ompl::base::PlannerTerminationCondition ompl::base::timedPlannerTerminationCondition(double duration)
{
    if (duration < 1.0)
        return PlannerTerminationCondition(boost::bind(&timePassed, time::now() + time::seconds(duration)));
    else
        return PlannerThreadedTerminationCondition(boost::bind(&timePassed, time::now() + time::seconds(duration)), std::min(duration / 100.0, 0.1));
}

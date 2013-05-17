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

void ompl::base::PlannerTerminationCondition::terminate(void) const
{
    terminate_ = true;
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

/// @cond IGNORE
namespace ompl
{
    namespace base
    {
        static bool plannerOrTerminationCondition(const PlannerTerminationCondition &c1, const PlannerTerminationCondition &c2)
        {
            return c1() || c2();
        }

        static bool plannerAndTerminationCondition(const PlannerTerminationCondition &c1, const PlannerTerminationCondition &c2)
        {
            return c1() && c2();
        }

        // return true if a certain point in time has passed
        static bool timePassed(const time::point &endTime)
        {
            return time::now() > endTime;
        }
    }
}
/// @endcond

ompl::base::PlannerOrTerminationCondition::PlannerOrTerminationCondition(const PlannerTerminationCondition &c1, const PlannerTerminationCondition &c2) :
    PlannerTerminationCondition(boost::bind(&plannerOrTerminationCondition, c1, c2))
{
}

ompl::base::PlannerAndTerminationCondition::PlannerAndTerminationCondition(const PlannerTerminationCondition &c1, const PlannerTerminationCondition &c2) :
    PlannerTerminationCondition(boost::bind(&plannerAndTerminationCondition, c1, c2))
{
}

bool ompl::base::PlannerThreadedTerminationCondition::eval(void) const
{
    return evalValue_;
}

void ompl::base::PlannerThreadedTerminationCondition::startEvalThread(void)
{
    if (!thread_)
        thread_ = new boost::thread(boost::bind(&PlannerThreadedTerminationCondition::periodicEval, this));
}

void ompl::base::PlannerThreadedTerminationCondition::stopEvalThread(void)
{
    terminate_ = true;
    if (thread_)
    {
        thread_->join();
        delete thread_;
        thread_ = NULL;
    }
}

bool ompl::base::PlannerThreadedTerminationCondition::computeEval(void)
{
    return fn_();
}

ompl::base::PlannerThreadedTerminationCondition::PlannerThreadedTerminationCondition(const PlannerTerminationConditionFn &fn, double period) :
    PlannerTerminationCondition(fn), thread_(NULL), evalValue_(terminate_), period_(period)
{
    startEvalThread();
}

ompl::base::PlannerThreadedTerminationCondition::PlannerThreadedTerminationCondition(const PlannerThreadedTerminationCondition &other) :
    PlannerTerminationCondition(other), thread_(NULL), evalValue_(other.evalValue_), period_(other.period_)
{
    startEvalThread();
}

ompl::base::PlannerThreadedTerminationCondition::~PlannerThreadedTerminationCondition(void)
{
    stopEvalThread();
}

ompl::base::PlannerThreadedTerminationCondition& ompl::base::PlannerThreadedTerminationCondition::operator=(const PlannerThreadedTerminationCondition &other)
{
    if (this != &other)
    {
	stopEvalThread();
	static_cast<PlannerTerminationCondition&>(*this) = static_cast<const PlannerTerminationCondition&>(other);
	thread_ = NULL;
	evalValue_ = other.evalValue_;
	period_ = other.period_;  
	startEvalThread();
    }
    return *this;
}

void ompl::base::PlannerThreadedTerminationCondition::periodicEval(void)
{
    // we want to check for termination at least once every ms;
    // even though we may evaluate the condition itself more rarely

    unsigned int count = 1;
    time::duration s = time::seconds(period_);
    if (period_ > 0.001)
    {
	count = 0.5 + period_ / 0.001;
	s = time::seconds(period_ / (double) count);
    }

    if (!(*this)())
	do
	{
	    evalValue_ = computeEval();
	    for (unsigned int i = 0 ; i < count ; ++i)
	    {
		if ((*this)())
		    break;
		boost::this_thread::sleep(s);
	    }
	} while (!(*this)());
}

ompl::base::PlannerTerminationCondition ompl::base::timedPlannerTerminationCondition(double duration)
{
    return PlannerTerminationCondition(boost::bind(&timePassed, time::now() + time::seconds(duration)));
}

ompl::base::PlannerThreadedTerminationCondition ompl::base::timedPlannerTerminationCondition(double duration, double interval)
{
    if (interval > duration)
        interval = duration;
    return PlannerThreadedTerminationCondition(boost::bind(&timePassed, time::now() + time::seconds(duration)), interval);
}

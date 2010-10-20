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

#include "ompl/base/GoalLazySamples.h"
#include "ompl/base/ScopedState.h"

ompl::base::GoalLazySamples::GoalLazySamples(const SpaceInformationPtr &si, const boost::function1<bool, State*> &samplerFunc) :
    GoalStates(si), samplerFunc_(samplerFunc), terminateSamplingThread_(false), samplingThread_(&GoalLazySamples::goalSamplingThread, this)
{
}

ompl::base::GoalLazySamples::~GoalLazySamples(void)
{
    terminateSamplingThread_ = true;
    samplingThread_.join();
}

void ompl::base::GoalLazySamples::goalSamplingThread(void)
{
    if (!terminateSamplingThread_ && samplerFunc_)
    {
	ScopedState<> s(si_);
	while (!terminateSamplingThread_ && samplerFunc_(s.get()))
	    addState(s.get());
    }
}

void ompl::base::GoalLazySamples::clear(void)
{
    boost::mutex::scoped_lock slock(lock_);
    GoalStates::clear();
}

double ompl::base::GoalLazySamples::distanceGoal(const State *st) const
{
    boost::mutex::scoped_lock slock(lock_);
    return GoalStates::distanceGoal(st);
}

void ompl::base::GoalLazySamples::sampleGoal(base::State *st) const
{
    boost::mutex::scoped_lock slock(lock_);
    GoalStates::sampleGoal(st);
}

void ompl::base::GoalLazySamples::addState(const State* st)
{
    boost::mutex::scoped_lock slock(lock_);
    GoalStates::addState(st);
}

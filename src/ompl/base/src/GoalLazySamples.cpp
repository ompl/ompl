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
#include "ompl/util/Time.h"

ompl::base::GoalLazySamples::GoalLazySamples(const SpaceInformationPtr &si, const GoalSamplingFn &samplerFunc, bool autoStart, double minDist) :
    GoalStates(si), samplerFunc_(samplerFunc), terminateSamplingThread_(false), samplingThread_(NULL), lastStateAdded_(false), samplingAttempts_(0), minDist_(minDist)
{
    type_ = GOAL_LAZY_SAMPLES;
    if (autoStart)
        startSampling();
}

ompl::base::GoalLazySamples::~GoalLazySamples(void)
{
    stopSampling();
}

void ompl::base::GoalLazySamples::startSampling(void)
{
    if (samplingThread_ == NULL)
    {
        terminateSamplingThread_ = false;
        samplingThread_ = new boost::thread(&GoalLazySamples::goalSamplingThread, this);
    }
}

void ompl::base::GoalLazySamples::stopSampling(void)
{
    if (isSampling())
    {
        terminateSamplingThread_ = true;
        samplingThread_->join();
        delete samplingThread_;
        samplingThread_ = NULL;
    }
}

void ompl::base::GoalLazySamples::goalSamplingThread(void)
{
    // wait for everything to be set up before performing computation
    while (!terminateSamplingThread_ && !si_->isSetup())
        boost::this_thread::sleep(time::seconds(0.01));

    if (!terminateSamplingThread_ && samplerFunc_)
    {
        ScopedState<> s(si_);
        while (!terminateSamplingThread_ && samplerFunc_(this, s.get()))
        {
            ++samplingAttempts_;
            if (si_->satisfiesBounds(s.get()) && si_->isValid(s.get()))
                addStateIfDifferent(s.get(), minDist_);
        }
    }
    terminateSamplingThread_ = true;
}

bool ompl::base::GoalLazySamples::isSampling(void) const
{
    return terminateSamplingThread_ == false && samplingThread_ != NULL;
}

bool ompl::base::GoalLazySamples::canSample(void) const
{
    return maxSampleCount() > 0 || isSampling();
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

void ompl::base::GoalLazySamples::setNewStateCallback(const NewStateCallbackFn &callback)
{
    callback_ = callback;
}

void ompl::base::GoalLazySamples::addState(const State* st)
{
    boost::mutex::scoped_lock slock(lock_);
    GoalStates::addState(st);
}

bool ompl::base::GoalLazySamples::addStateIfDifferent(const State* st, double minDistance)
{
    const base::State *newState = NULL;
    {
        boost::mutex::scoped_lock slock(lock_);
        if (GoalStates::distanceGoal(st) > minDistance)
        {
            GoalStates::addState(st);
            lastStateAdded_ = true;
            if (callback_)
                newState = states_.back();
        }
        else
            lastStateAdded_ = false;
    }

    // the lock is released at this; if needed, issue a call to the callback
    if (newState)
        callback_(newState);
    return lastStateAdded_;
}

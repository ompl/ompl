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

#include "ompl/base/goals/GoalLazySamples.h"
#include "ompl/base/ScopedState.h"
#include "ompl/util/Time.h"

ompl::base::GoalLazySamples::GoalLazySamples(const SpaceInformationPtr &si, const GoalSamplingFn &samplerFunc, bool autoStart, double minDist) :
    GoalStates(si), samplerFunc_(samplerFunc), terminateSamplingThread_(false), samplingThread_(NULL), samplingAttempts_(0), minDist_(minDist)
{
    type_ = GOAL_LAZY_SAMPLES;
    if (autoStart)
        startSampling();
}

ompl::base::GoalLazySamples::~GoalLazySamples()
{
    stopSampling();
}

void ompl::base::GoalLazySamples::startSampling()
{
    if (samplingThread_ == NULL)
    {
        OMPL_DEBUG("Starting goal sampling thread");
        terminateSamplingThread_ = false;
        samplingThread_ = new boost::thread(&GoalLazySamples::goalSamplingThread, this);
    }
}

void ompl::base::GoalLazySamples::stopSampling()
{
    if (isSampling())
    {
        OMPL_DEBUG("Attempting to stop goal sampling thread...");
        terminateSamplingThread_ = true;
        samplingThread_->join();
        delete samplingThread_;
        samplingThread_ = NULL;
    }
    else
        if (samplingThread_)
        { // join a finished thread
            samplingThread_->join();
            delete samplingThread_;
            samplingThread_ = NULL;
        }
}

void ompl::base::GoalLazySamples::goalSamplingThread()
{
    if (!si_->isSetup())
    {
        OMPL_DEBUG("Waiting for space information to be set up before the sampling thread can begin computation...");
        // wait for everything to be set up before performing computation
        while (!terminateSamplingThread_ && !si_->isSetup())
            boost::this_thread::sleep(time::seconds(0.01));
    }
    unsigned int prevsa = samplingAttempts_;
    if (!terminateSamplingThread_ && samplerFunc_)
    {
        OMPL_DEBUG("Beginning sampling thread computation");
        ScopedState<> s(si_);
        while (!terminateSamplingThread_ && samplerFunc_(this, s.get()))
        {
            ++samplingAttempts_;
            if (si_->satisfiesBounds(s.get()) && si_->isValid(s.get()))
                addStateIfDifferent(s.get(), minDist_);
        }
    }
    else
        OMPL_WARN("Goal sampling thread never did any work.%s",
                  samplerFunc_ ? (si_->isSetup() ? "" : " Space information not set up.") : " No sampling function set.");
    terminateSamplingThread_ = true;
    OMPL_DEBUG("Stopped goal sampling thread after %u sampling attempts", samplingAttempts_ - prevsa);
}

bool ompl::base::GoalLazySamples::isSampling() const
{
    return terminateSamplingThread_ == false && samplingThread_ != NULL;
}

bool ompl::base::GoalLazySamples::couldSample() const
{
    return canSample() || isSampling();
}

void ompl::base::GoalLazySamples::clear()
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

void ompl::base::GoalLazySamples::addState(const State *st)
{
    boost::mutex::scoped_lock slock(lock_);
    GoalStates::addState(st);
}

const ompl::base::State* ompl::base::GoalLazySamples::getState(unsigned int index) const
{
    boost::mutex::scoped_lock slock(lock_);
    return GoalStates::getState(index);
}

bool ompl::base::GoalLazySamples::hasStates() const
{
    boost::mutex::scoped_lock slock(lock_);
    return GoalStates::hasStates();
}

std::size_t ompl::base::GoalLazySamples::getStateCount() const
{
    boost::mutex::scoped_lock slock(lock_);
    return GoalStates::getStateCount();
}

bool ompl::base::GoalLazySamples::addStateIfDifferent(const State *st, double minDistance)
{
    const base::State *newState = NULL;
    bool added = false;
    {
        boost::mutex::scoped_lock slock(lock_);
        if (GoalStates::distanceGoal(st) > minDistance)
        {
            GoalStates::addState(st);
            added = true;
            if (callback_)
                newState = states_.back();
        }
    }

    // the lock is released at this; if needed, issue a call to the callback
    if (newState)
        callback_(newState);
    return added;
}

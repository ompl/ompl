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

#include <utility>

#include "ompl/base/ScopedState.h"
#include "ompl/base/goals/GoalLazySamples.h"
#include "ompl/util/Time.h"

ompl::base::GoalLazySamples::GoalLazySamples(const SpaceInformationPtr &si, GoalSamplingFn samplerFunc, bool autoStart,
                                             double minDist)
  : GoalStates(si)
  , samplerFunc_(std::move(samplerFunc))
  , terminateSamplingThread_(false)
  , samplingThread_(nullptr)
  , samplingAttempts_(0)
  , minDist_(minDist)
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
    std::lock_guard<std::mutex> slock(lock_);
    if (samplingThread_ == nullptr)
    {
        OMPL_DEBUG("Starting goal sampling thread");
        terminateSamplingThread_ = false;
        samplingThread_ = new std::thread(&GoalLazySamples::goalSamplingThread, this);
    }
}

void ompl::base::GoalLazySamples::stopSampling()
{
    /* Set termination flag */
    {
        std::lock_guard<std::mutex> slock(lock_);
        if (!terminateSamplingThread_)
        {
            OMPL_DEBUG("Attempting to stop goal sampling thread...");
            terminateSamplingThread_ = true;
        }
    }

    /* Join thread */
    if (samplingThread_ != nullptr)
    {
        samplingThread_->join();
        delete samplingThread_;
        samplingThread_ = nullptr;
    }
}

void ompl::base::GoalLazySamples::goalSamplingThread()
{
    {
        /* Wait for startSampling() to finish assignment
         * samplingThread_ */
        std::lock_guard<std::mutex> slock(lock_);
    }

    if (!si_->isSetup())  // this looks racy
    {
        OMPL_DEBUG("Waiting for space information to be set up before the sampling thread can begin computation...");
        // wait for everything to be set up before performing computation
        while (!terminateSamplingThread_ && !si_->isSetup())
            std::this_thread::sleep_for(time::seconds(0.01));
    }
    unsigned int prevsa = samplingAttempts_;
    if (isSampling() && samplerFunc_)
    {
        OMPL_DEBUG("Beginning sampling thread computation");
        ScopedState<> s(si_);
        while (isSampling() && samplerFunc_(this, s.get()))
        {
            ++samplingAttempts_;
            if (si_->satisfiesBounds(s.get()) && si_->isValid(s.get()))
            {
                OMPL_DEBUG("Adding goal state");
                addStateIfDifferent(s.get(), minDist_);
            }
            else
            {
                OMPL_DEBUG("Invalid goal candidate");
            }
        }
    }
    else
        OMPL_WARN("Goal sampling thread never did any work.%s",
                  samplerFunc_ ? (si_->isSetup() ? "" : " Space information not set up.") : " No sampling function "
                                                                                            "set.");
    {
        std::lock_guard<std::mutex> slock(lock_);
        terminateSamplingThread_ = true;
    }

    OMPL_DEBUG("Stopped goal sampling thread after %u sampling attempts", samplingAttempts_ - prevsa);
}

bool ompl::base::GoalLazySamples::isSampling() const
{
    std::lock_guard<std::mutex> slock(lock_);
    return !terminateSamplingThread_ && samplingThread_ != nullptr;
}

bool ompl::base::GoalLazySamples::couldSample() const
{
    return canSample() || isSampling();
}

void ompl::base::GoalLazySamples::clear()
{
    std::lock_guard<std::mutex> slock(lock_);
    GoalStates::clear();
}

double ompl::base::GoalLazySamples::distanceGoal(const State *st) const
{
    std::lock_guard<std::mutex> slock(lock_);
    return GoalStates::distanceGoal(st);
}

void ompl::base::GoalLazySamples::sampleGoal(base::State *st) const
{
    std::lock_guard<std::mutex> slock(lock_);
    GoalStates::sampleGoal(st);
}

void ompl::base::GoalLazySamples::setNewStateCallback(const NewStateCallbackFn &callback)
{
    callback_ = callback;
}

void ompl::base::GoalLazySamples::addState(const State *st)
{
    std::lock_guard<std::mutex> slock(lock_);
    GoalStates::addState(st);
}

const ompl::base::State *ompl::base::GoalLazySamples::getState(unsigned int index) const
{
    std::lock_guard<std::mutex> slock(lock_);
    return GoalStates::getState(index);
}

bool ompl::base::GoalLazySamples::hasStates() const
{
    std::lock_guard<std::mutex> slock(lock_);
    return GoalStates::hasStates();
}

std::size_t ompl::base::GoalLazySamples::getStateCount() const
{
    std::lock_guard<std::mutex> slock(lock_);
    return GoalStates::getStateCount();
}

unsigned int ompl::base::GoalLazySamples::maxSampleCount() const
{
    std::lock_guard<std::mutex> slock(lock_);
    return GoalStates::maxSampleCount();
}

bool ompl::base::GoalLazySamples::addStateIfDifferent(const State *st, double minDistance)
{
    const base::State *newState = nullptr;
    bool added = false;
    {
        std::lock_guard<std::mutex> slock(lock_);
        if (GoalStates::distanceGoal(st) > minDistance)
        {
            GoalStates::addState(st);
            added = true;
            if (callback_)
                newState = states_.back();
        }
    }

    // the lock is released at this; if needed, issue a call to the callback
    if (newState != nullptr)
        callback_(newState);
    return added;
}

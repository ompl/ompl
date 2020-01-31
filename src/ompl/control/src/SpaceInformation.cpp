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

#include "ompl/control/SpaceInformation.h"
#include "ompl/control/SimpleDirectedControlSampler.h"
#include "ompl/control/SteeredControlSampler.h"
#include "ompl/util/Exception.h"
#include <cassert>
#include <utility>
#include <limits>

ompl::control::SpaceInformation::SpaceInformation(
    const base::StateSpacePtr &stateSpace, ControlSpacePtr controlSpace)
    : base::SpaceInformation(stateSpace), controlSpace_(std::move(controlSpace))
{
    declareParams();
}

void ompl::control::SpaceInformation::declareParams()
{
    params_.declareParam<unsigned int>("min_control_duration",
                                 [this](unsigned int n) { setMinControlDuration(n); },
                                 [this] { return getMinControlDuration(); });
    params_.declareParam<unsigned int>("max_control_duration",
                                 [this](unsigned int n) { setMaxControlDuration(n); },
                                 [this] { return getMaxControlDuration(); });
    params_.declareParam<double>("propagation_step_size",
                                 [this](double s) { setPropagationStepSize(s); },
                                 [this] { return getPropagationStepSize(); });
}

void ompl::control::SpaceInformation::setup()
{
    base::SpaceInformation::setup();
    declareParams(); // calling base::SpaceInformation::setup() clears the params
    if (!statePropagator_)
        throw Exception("State propagator not defined");
    if (minSteps_ > maxSteps_)
        throw Exception("The minimum number of steps cannot be larger than the maximum number of steps");
    if (minSteps_ == 0 && maxSteps_ == 0)
    {
        minSteps_ = 1;
        maxSteps_ = 10;
        OMPL_WARN("Assuming propagation will always have between %d and %d steps", minSteps_, maxSteps_);
    }
    if (minSteps_ < 1)
        throw Exception("The minimum number of steps must be at least 1");

    if (stepSize_ < std::numeric_limits<double>::epsilon())
    {
        stepSize_ = getStateValidityCheckingResolution() * getMaximumExtent();
        if (stepSize_ < std::numeric_limits<double>::epsilon())
            throw Exception("The propagation step size must be larger than 0");
        OMPL_WARN("The propagation step size is assumed to be %f", stepSize_);
    }

    controlSpace_->setup();
    if (controlSpace_->getDimension() <= 0)
        throw Exception("The dimension of the control space we plan in must be > 0");
}

ompl::control::DirectedControlSamplerPtr ompl::control::SpaceInformation::allocDirectedControlSampler() const
{
    if (dcsa_)
        return dcsa_(this);
    if (statePropagator_->canSteer())
        return std::make_shared<SteeredControlSampler>(this);
    else
        return std::make_shared<SimpleDirectedControlSampler>(this);
}

void ompl::control::SpaceInformation::setDirectedControlSamplerAllocator(const DirectedControlSamplerAllocator &dcsa)
{
    dcsa_ = dcsa;
    setup_ = false;
}

void ompl::control::SpaceInformation::clearDirectedSamplerAllocator()
{
    dcsa_ = DirectedControlSamplerAllocator();
    setup_ = false;
}

void ompl::control::SpaceInformation::setStatePropagator(const StatePropagatorFn &fn)
{
    class FnStatePropagator : public StatePropagator
    {
    public:
        FnStatePropagator(SpaceInformation *si, StatePropagatorFn fn) : StatePropagator(si), fn_(std::move(fn))
        {
        }

        void propagate(const base::State *state, const Control *control, const double duration,
                       base::State *result) const override
        {
            fn_(state, control, duration, result);
        }

    protected:
        StatePropagatorFn fn_;
    };

    setStatePropagator(std::make_shared<FnStatePropagator>(this, fn));
}

void ompl::control::SpaceInformation::setStatePropagator(const StatePropagatorPtr &sp)
{
    statePropagator_ = sp;
}

bool ompl::control::SpaceInformation::canPropagateBackward() const
{
    return statePropagator_->canPropagateBackward();
}

void ompl::control::SpaceInformation::propagate(const base::State *state, const Control *control, int steps,
                                                base::State *result) const
{
    if (steps == 0)
    {
        if (result != state)
            copyState(result, state);
    }
    else
    {
        double signedStepSize = steps > 0 ? stepSize_ : -stepSize_;
        steps = abs(steps);

        statePropagator_->propagate(state, control, signedStepSize, result);
        for (int i = 1; i < steps; ++i)
            statePropagator_->propagate(result, control, signedStepSize, result);
    }
}

unsigned int ompl::control::SpaceInformation::propagateWhileValid(const base::State *state, const Control *control,
                                                                  int steps, base::State *result) const
{
    if (steps == 0)
    {
        if (result != state)
            copyState(result, state);
        return 0;
    }

    double signedStepSize = steps > 0 ? stepSize_ : -stepSize_;
    steps = abs(steps);

    // perform the first step of propagation
    statePropagator_->propagate(state, control, signedStepSize, result);

    // if we found a valid state after one step, we can go on
    if (isValid(result))
    {
        base::State *temp1 = result;
        base::State *temp2 = allocState();
        base::State *toDelete = temp2;
        unsigned int r = steps;

        // for the remaining number of steps
        for (int i = 1; i < steps; ++i)
        {
            statePropagator_->propagate(temp1, control, signedStepSize, temp2);
            if (isValid(temp2))
                std::swap(temp1, temp2);
            else
            {
                // the last valid state is temp1;
                r = i;
                break;
            }
        }

        // if we finished the for-loop without finding an invalid state, the last valid state is temp1
        // make sure result contains that information
        if (result != temp1)
            copyState(result, temp1);

        // free the temporary memory
        freeState(toDelete);

        return r;
    }
    // if the first propagation step produced an invalid step, return 0 steps
    // the last valid state is the starting one (assumed to be valid)
    if (result != state)
        copyState(result, state);
    return 0;
}

void ompl::control::SpaceInformation::propagate(const base::State *state, const Control *control, int steps,
                                                std::vector<base::State *> &result, bool alloc) const
{
    double signedStepSize = steps > 0 ? stepSize_ : -stepSize_;
    steps = abs(steps);

    if (alloc)
    {
        result.resize(steps);
        for (auto &i : result)
            i = allocState();
    }
    else
    {
        if (result.empty())
            return;
        steps = std::min(steps, (int)result.size());
    }

    int st = 0;

    if (st < steps)
    {
        statePropagator_->propagate(state, control, signedStepSize, result[st]);
        ++st;

        while (st < steps)
        {
            statePropagator_->propagate(result[st - 1], control, signedStepSize, result[st]);
            ++st;
        }
    }
}

unsigned int ompl::control::SpaceInformation::propagateWhileValid(const base::State *state, const Control *control,
                                                                  int steps, std::vector<base::State *> &result,
                                                                  bool alloc) const
{
    double signedStepSize = steps > 0 ? stepSize_ : -stepSize_;
    steps = abs(steps);

    if (alloc)
        result.resize(steps);
    else
    {
        if (result.empty())
            return 0;
        steps = std::min(steps, (int)result.size());
    }

    int st = 0;

    if (st < steps)
    {
        if (alloc)
            result[st] = allocState();
        statePropagator_->propagate(state, control, signedStepSize, result[st]);

        if (isValid(result[st]))
        {
            ++st;
            while (st < steps)
            {
                if (alloc)
                    result[st] = allocState();
                statePropagator_->propagate(result[st - 1], control, signedStepSize, result[st]);

                if (!isValid(result[st]))
                {
                    if (alloc)
                    {
                        freeState(result[st]);
                        result.resize(st);
                    }
                    break;
                }
                ++st;
            }
        }
        else
        {
            if (alloc)
            {
                freeState(result[st]);
                result.resize(st);
            }
        }
    }

    return st;
}

void ompl::control::SpaceInformation::printSettings(std::ostream &out) const
{
    base::SpaceInformation::printSettings(out);
    out << "  - control space:" << std::endl;
    controlSpace_->printSettings(out);
    out << "  - can propagate backward: " << (canPropagateBackward() ? "yes" : "no") << std::endl;
    out << "  - propagation step size: " << stepSize_ << std::endl;
    out << "  - propagation duration: [" << minSteps_ << ", " << maxSteps_ << "]" << std::endl;
}

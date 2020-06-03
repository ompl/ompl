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

#include "ompl/base/SpaceInformation.h"
#include <cassert>
#include <queue>
#include <utility>
#include "ompl/base/DiscreteMotionValidator.h"
#include "ompl/base/samplers/UniformValidStateSampler.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/util/Exception.h"
#include "ompl/util/Time.h"

ompl::base::SpaceInformation::SpaceInformation(StateSpacePtr space) : stateSpace_(std::move(space)), setup_(false)
{
    if (!stateSpace_)
        throw Exception("Invalid space definition");
    setDefaultMotionValidator();
    params_.include(stateSpace_->params());
}

void ompl::base::SpaceInformation::setup()
{
    if (!stateValidityChecker_)
    {
        stateValidityChecker_ = std::make_shared<AllValidStateValidityChecker>(this);
        OMPL_WARN("State validity checker not set! No collision checking is performed");
    }

    if (!motionValidator_)
        setDefaultMotionValidator();

    stateSpace_->setup();
    if (stateSpace_->getDimension() <= 0)
        throw Exception("The dimension of the state space we plan in must be > 0");

    params_.clear();
    params_.include(stateSpace_->params());

    setup_ = true;
}

bool ompl::base::SpaceInformation::isSetup() const
{
    return setup_;
}

void ompl::base::SpaceInformation::setStateValidityChecker(const StateValidityCheckerFn &svc)
{
    class FnStateValidityChecker : public StateValidityChecker
    {
    public:
        FnStateValidityChecker(SpaceInformation *si, StateValidityCheckerFn fn)
          : StateValidityChecker(si), fn_(std::move(fn))
        {
        }

        bool isValid(const State *state) const override
        {
            return fn_(state);
        }

    protected:
        StateValidityCheckerFn fn_;
    };

    if (!svc)
        throw Exception("Invalid function definition for state validity checking");

    setStateValidityChecker(std::make_shared<FnStateValidityChecker>(this, svc));
}

void ompl::base::SpaceInformation::setDefaultMotionValidator()
{
    if (dynamic_cast<ReedsSheppStateSpace *>(stateSpace_.get()))
        motionValidator_ = std::make_shared<ReedsSheppMotionValidator>(this);
    else if (dynamic_cast<DubinsStateSpace *>(stateSpace_.get()))
        motionValidator_ = std::make_shared<DubinsMotionValidator>(this);
    else if (dynamic_cast<ConstrainedStateSpace *>(stateSpace_.get()))
        motionValidator_ = std::make_shared<ConstrainedMotionValidator>(this);
    else
        motionValidator_ = std::make_shared<DiscreteMotionValidator>(this);
}

void ompl::base::SpaceInformation::setValidStateSamplerAllocator(const ValidStateSamplerAllocator &vssa)
{
    vssa_ = vssa;
    setup_ = false;
}

void ompl::base::SpaceInformation::clearValidStateSamplerAllocator()
{
    vssa_ = ValidStateSamplerAllocator();
    setup_ = false;
}

unsigned int ompl::base::SpaceInformation::randomBounceMotion(const StateSamplerPtr &sss, const State *start,
                                                              unsigned int steps, std::vector<State *> &states,
                                                              bool alloc) const
{
    if (alloc)
    {
        states.resize(steps);
        for (unsigned int i = 0; i < steps; ++i)
            states[i] = allocState();
    }
    else if (states.size() < steps)
        steps = states.size();

    const State *prev = start;
    std::pair<State *, double> lastValid;
    unsigned int j = 0;
    for (unsigned int i = 0; i < steps; ++i)
    {
        sss->sampleUniform(states[j]);
        lastValid.first = states[j];
        if (checkMotion(prev, states[j], lastValid) || lastValid.second > std::numeric_limits<double>::epsilon())
            prev = states[j++];
    }

    return j;
}

bool ompl::base::SpaceInformation::searchValidNearby(const ValidStateSamplerPtr &sampler, State *state,
                                                     const State *near, double distance) const
{
    if (state != near)
        copyState(state, near);

    // fix bounds, if needed
    if (!satisfiesBounds(state))
        enforceBounds(state);

    bool result = isValid(state);

    if (!result)
    {
        // try to find a valid state nearby
        State *temp = cloneState(state);
        result = sampler->sampleNear(state, temp, distance);
        freeState(temp);
    }

    return result;
}

bool ompl::base::SpaceInformation::searchValidNearby(State *state, const State *near, double distance,
                                                     unsigned int attempts) const
{
    if (satisfiesBounds(near) && isValid(near))
    {
        if (state != near)
            copyState(state, near);
        return true;
    }
    else
    {
        // try to find a valid state nearby
        auto uvss = std::make_shared<UniformValidStateSampler>(this);
        uvss->setNrAttempts(attempts);
        return searchValidNearby(uvss, state, near, distance);
    }
}

unsigned int ompl::base::SpaceInformation::getMotionStates(const State *s1, const State *s2,
                                                           std::vector<State *> &states, unsigned int count,
                                                           bool endpoints, bool alloc) const
{
    // add 1 to the number of states we want to add between s1 & s2. This gives us the number of segments to split the
    // motion into
    count++;

    if (count < 2)
    {
        unsigned int added = 0;

        // if they want endpoints, then at most endpoints are included
        if (endpoints)
        {
            if (alloc)
            {
                states.resize(2);
                states[0] = allocState();
                states[1] = allocState();
            }
            if (states.size() > 0)
            {
                copyState(states[0], s1);
                added++;
            }

            if (states.size() > 1)
            {
                copyState(states[1], s2);
                added++;
            }
        }
        else if (alloc)
            states.resize(0);
        return added;
    }

    if (alloc)
    {
        states.resize(count + (endpoints ? 1 : -1));
        if (endpoints)
            states[0] = allocState();
    }

    unsigned int added = 0;

    if (endpoints && states.size() > 0)
    {
        copyState(states[0], s1);
        added++;
    }

    /* find the states in between */
    for (unsigned int j = 1; j < count && added < states.size(); ++j)
    {
        if (alloc)
            states[added] = allocState();
        stateSpace_->interpolate(s1, s2, (double)j / (double)count, states[added]);
        added++;
    }

    if (added < states.size() && endpoints)
    {
        if (alloc)
            states[added] = allocState();
        copyState(states[added], s2);
        added++;
    }

    return added;
}

bool ompl::base::SpaceInformation::checkMotion(const std::vector<State *> &states, unsigned int count,
                                               unsigned int &firstInvalidStateIndex) const
{
    assert(states.size() >= count);
    for (unsigned int i = 0; i < count; ++i)
        if (!isValid(states[i]))
        {
            firstInvalidStateIndex = i;
            return false;
        }
    return true;
}

bool ompl::base::SpaceInformation::checkMotion(const std::vector<State *> &states, unsigned int count) const
{
    assert(states.size() >= count);
    if (count > 0)
    {
        if (count > 1)
        {
            if (!isValid(states.front()))
                return false;
            if (!isValid(states[count - 1]))
                return false;

            // we have 2 or more states, and the first and last states are valid

            if (count > 2)
            {
                std::queue<std::pair<int, int>> pos;
                pos.emplace(0, count - 1);

                while (!pos.empty())
                {
                    std::pair<int, int> x = pos.front();

                    int mid = (x.first + x.second) / 2;
                    if (!isValid(states[mid]))
                        return false;

                    pos.pop();

                    if (x.first < mid - 1)
                        pos.emplace(x.first, mid);
                    if (x.second > mid + 1)
                        pos.emplace(mid, x.second);
                }
            }
        }
        else
            return isValid(states.front());
    }
    return true;
}

ompl::base::ValidStateSamplerPtr ompl::base::SpaceInformation::allocValidStateSampler() const
{
    if (vssa_)
        return vssa_(this);
    else
        return std::make_shared<UniformValidStateSampler>(this);
}

double ompl::base::SpaceInformation::probabilityOfValidState(unsigned int attempts) const
{
    if (attempts == 0)
        return 0.0;

    unsigned int valid = 0;
    unsigned int invalid = 0;

    StateSamplerPtr ss = allocStateSampler();
    State *s = allocState();

    for (unsigned int i = 0; i < attempts; ++i)
    {
        ss->sampleUniform(s);
        if (isValid(s))
            ++valid;
        else
            ++invalid;
    }

    freeState(s);

    return (double)valid / (double)(valid + invalid);
}

double ompl::base::SpaceInformation::averageValidMotionLength(unsigned int attempts) const
{
    // take the square root here because we in fact have a nested for loop
    // where each loop executes #attempts steps (the sample() function of the UniformValidStateSampler if a for loop
    // too)
    attempts = std::max((unsigned int)floor(sqrt((double)attempts) + 0.5), 2u);

    StateSamplerPtr ss = allocStateSampler();
    auto uvss(std::make_shared<UniformValidStateSampler>(this));
    uvss->setNrAttempts(attempts);

    State *s1 = allocState();
    State *s2 = allocState();

    std::pair<State *, double> lastValid;
    lastValid.first = nullptr;

    double d = 0.0;
    unsigned int count = 0;
    for (unsigned int i = 0; i < attempts; ++i)
        if (uvss->sample(s1))
        {
            ++count;
            ss->sampleUniform(s2);
            if (checkMotion(s1, s2, lastValid))
                d += distance(s1, s2);
            else
                d += distance(s1, s2) * lastValid.second;
        }

    freeState(s2);
    freeState(s1);

    if (count > 0)
        return d / (double)count;
    else
        return 0.0;
}

void ompl::base::SpaceInformation::samplesPerSecond(double &uniform, double &near, double &gaussian,
                                                    unsigned int attempts) const
{
    StateSamplerPtr ss = allocStateSampler();
    std::vector<State *> states(attempts + 1);
    allocStates(states);

    time::point start = time::now();
    for (unsigned int i = 0; i < attempts; ++i)
        ss->sampleUniform(states[i]);
    uniform = (double)attempts / time::seconds(time::now() - start);

    double d = getMaximumExtent() / 10.0;
    ss->sampleUniform(states[attempts]);

    start = time::now();
    for (unsigned int i = 1; i <= attempts; ++i)
        ss->sampleUniformNear(states[i - 1], states[i], d);
    near = (double)attempts / time::seconds(time::now() - start);

    start = time::now();
    for (unsigned int i = 1; i <= attempts; ++i)
        ss->sampleGaussian(states[i - 1], states[i], d);
    gaussian = (double)attempts / time::seconds(time::now() - start);

    freeStates(states);
}

void ompl::base::SpaceInformation::printSettings(std::ostream &out) const
{
    out << "Settings for the state space '" << stateSpace_->getName() << "'" << std::endl;
    out << "  - state validity check resolution: " << (getStateValidityCheckingResolution() * 100.0) << '%'
        << std::endl;
    out << "  - valid segment count factor: " << stateSpace_->getValidSegmentCountFactor() << std::endl;
    out << "  - state space:" << std::endl;
    stateSpace_->printSettings(out);
    out << std::endl << "Declared parameters:" << std::endl;
    params_.print(out);
    ValidStateSamplerPtr vss = allocValidStateSampler();
    out << "Valid state sampler named " << vss->getName() << " with parameters:" << std::endl;
    vss->params().print(out);
}

void ompl::base::SpaceInformation::printProperties(std::ostream &out) const
{
    out << "Properties of the state space '" << stateSpace_->getName() << "'" << std::endl;
    out << "  - signature: ";
    std::vector<int> sig;
    stateSpace_->computeSignature(sig);
    for (int i : sig)
        out << i << " ";
    out << std::endl;
    out << "  - dimension: " << stateSpace_->getDimension() << std::endl;
    out << "  - extent: " << stateSpace_->getMaximumExtent() << std::endl;
    if (isSetup())
    {
        bool result = true;
        try
        {
            stateSpace_->sanityChecks();
        }
        catch (Exception &e)
        {
            result = false;
            out << std::endl
                << "  - SANITY CHECKS FOR STATE SPACE ***DID NOT PASS*** (" << e.what() << ")" << std::endl
                << std::endl;
            OMPL_ERROR(e.what());
        }
        if (result)
            out << "  - sanity checks for state space passed" << std::endl;
        out << "  - probability of valid states: " << probabilityOfValidState(magic::TEST_STATE_COUNT) << std::endl;
        out << "  - average length of a valid motion: " << averageValidMotionLength(magic::TEST_STATE_COUNT)
            << std::endl;
        double uniform, near, gaussian;
        samplesPerSecond(uniform, near, gaussian, magic::TEST_STATE_COUNT);
        out << "  - average number of samples drawn per second: sampleUniform()=" << uniform
            << " sampleUniformNear()=" << near << " sampleGaussian()=" << gaussian << std::endl;
    }
    else
        out << "Call setup() before to get more information" << std::endl;
}

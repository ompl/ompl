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
#include "ompl/base/samplers/UniformValidStateSampler.h"
#include "ompl/base/DiscreteMotionValidator.h"
#include "ompl/util/Exception.h"
#include <queue>
#include <cassert>

ompl::base::SpaceInformation::SpaceInformation(const StateSpacePtr &space) :
    stateSpace_(space), motionValidator_(new DiscreteMotionValidator(this)), setup_(false), msg_("SpaceInformation")
{
    if (!stateSpace_)
        throw Exception("Invalid space definition");
}

void ompl::base::SpaceInformation::setup(void)
{
    if (!stateValidityChecker_)
    {
        stateValidityChecker_.reset(new AllValidStateValidityChecker(this));
        msg_.warn("State validity checker not set! No collision checking is performed");
    }

    if (!motionValidator_)
        motionValidator_.reset(new DiscreteMotionValidator(this));

    stateSpace_->setup();
    if (stateSpace_->getDimension() <= 0)
        throw Exception("The dimension of the state space we plan in must be > 0");

    setup_ = true;
}

bool ompl::base::SpaceInformation::isSetup(void) const
{
    return setup_;
}

void ompl::base::SpaceInformation::setStateValidityChecker(const StateValidityCheckerFn &svc)
{
    class BoostFnStateValidityChecker : public StateValidityChecker
    {
    public:

        BoostFnStateValidityChecker(SpaceInformation* si,
                                    const StateValidityCheckerFn &fn) : StateValidityChecker(si), fn_(fn)
        {
        }

        virtual bool isValid(const State *state) const
        {
            return fn_(state);
        }

    protected:

        StateValidityCheckerFn fn_;
    };

    if (!svc)
        throw Exception("Invalid function definition for state validity checking");

    setStateValidityChecker(StateValidityCheckerPtr(dynamic_cast<StateValidityChecker*>(new BoostFnStateValidityChecker(this, svc))));
}

unsigned int ompl::base::SpaceInformation::randomBounceMotion(const StateSamplerPtr &sss, const State *start, unsigned int steps, std::vector<State*> &states, bool alloc) const
{
    if (alloc)
    {
        states.resize(steps);
        for (unsigned int i = 0 ; i < steps ; ++i)
            states[i] = allocState();
    }
    else
        if (states.size() < steps)
            steps = states.size();

    const State *prev = start;
    std::pair<State*, double> lastValid;

    unsigned int j = 0;
    for (unsigned int i = 0 ; i < steps ; ++i)
    {
        sss->sampleUniform(states[j]);
        lastValid.first = states[j];
        if (checkMotion(prev, states[j], lastValid) || lastValid.second > std::numeric_limits<double>::epsilon())
            prev = states[j++];
    }
    return j;
}

bool ompl::base::SpaceInformation::searchValidNearby(const ValidStateSamplerPtr &sampler, State *state, const State *near, double distance) const
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

bool ompl::base::SpaceInformation::searchValidNearby(State *state, const State *near, double distance, unsigned int attempts) const
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
        UniformValidStateSampler *uvss = new UniformValidStateSampler(this);
        uvss->setNrAttempts(attempts);
        return searchValidNearby(ValidStateSamplerPtr(uvss), state, near, distance);
    }
}

unsigned int ompl::base::SpaceInformation::getMotionStates(const State *s1, const State *s2, std::vector<State*> &states, unsigned int count, bool endpoints, bool alloc) const
{
    // add 1 to the number of states we want to add between s1 & s2. This gives us the number of segments to split the motion into
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
        else
            if (alloc)
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
    for (unsigned int j = 1 ; j < count && added < states.size() ; ++j)
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


bool ompl::base::SpaceInformation::checkMotion(const std::vector<State*> &states, unsigned int count, unsigned int &firstInvalidStateIndex) const
{
    assert(states.size() >= count);
    for (unsigned int i = 0 ; i < count ; ++i)
        if (!isValid(states[i]))
        {
            firstInvalidStateIndex = i;
            return false;
        }
    return true;
}

bool ompl::base::SpaceInformation::checkMotion(const std::vector<State*> &states, unsigned int count) const
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
                std::queue< std::pair<int, int> > pos;
                pos.push(std::make_pair(0, count - 1));

                while (!pos.empty())
                {
                    std::pair<int, int> x = pos.front();

                    int mid = (x.first + x.second) / 2;
                    if (!isValid(states[mid]))
                        return false;

                    if (x.first < mid - 1)
                        pos.push(std::make_pair(x.first, mid));
                    if (x.second > mid + 1)
                        pos.push(std::make_pair(mid, x.second));
                }
            }
        }
        else
            return isValid(states.front());
    }
    return true;
}

ompl::base::ValidStateSamplerPtr ompl::base::SpaceInformation::allocValidStateSampler(void) const
{
    if (vssa_)
        return vssa_(this);
    else
        return ValidStateSamplerPtr(new UniformValidStateSampler(this));
}

double ompl::base::SpaceInformation::probabilityOfValidState(unsigned int attempts) const
{
    unsigned int valid = 0;
    unsigned int invalid = 0;

    StateSamplerPtr ss = allocStateSampler();
    State *s = allocState();

    for (unsigned int i = 0 ; i < attempts ; ++i)
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
    StateSamplerPtr ss = allocStateSampler();
    UniformValidStateSampler *uvss = new UniformValidStateSampler(this);
    uvss->setNrAttempts(attempts);

    State *s1 = allocState();
    State *s2 = allocState();

    std::pair<State*, double> lastValid;
    lastValid.first = NULL;

    double d = 0.0;
    unsigned int count = 0;
    for (unsigned int i = 0 ; i < attempts ; ++i)
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
    delete uvss;

    if (count > 0)
        return d / (double)count;
    else
        return 0.0;
}

void ompl::base::SpaceInformation::printSettings(std::ostream &out) const
{
    out << "State space settings:" << std::endl;
    out << "  - dimension: " << stateSpace_->getDimension() << std::endl;
    out << "  - extent: " << stateSpace_->getMaximumExtent() << std::endl;
    out << "  - state validity check resolution: " << (getStateValidityCheckingResolution() * 100.0) << '%' << std::endl;
    out << "  - valid segment count factor: " << stateSpace_->getValidSegmentCountFactor() << std::endl;
    out << "  - state space:" << std::endl;
    stateSpace_->printSettings(out);
}

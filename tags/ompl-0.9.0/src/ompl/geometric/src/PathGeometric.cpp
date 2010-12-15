/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/samplers/UniformValidStateSampler.h"
#include "ompl/base/ScopedState.h"
#include <algorithm>
#include <cmath>

ompl::geometric::PathGeometric::PathGeometric(const PathGeometric &path) : base::Path(path.si_)
{
    copyFrom(path);
}

ompl::geometric::PathGeometric& ompl::geometric::PathGeometric::operator=(const PathGeometric &other)
{
    freeMemory();
    si_ = other.si_;
    copyFrom(other);
    return *this;
}

void ompl::geometric::PathGeometric::copyFrom(const PathGeometric &other)
{
    states.resize(other.states.size());
    for (unsigned int i = 0 ; i < states.size() ; ++i)
        states[i] = si_->cloneState(other.states[i]);
}

void ompl::geometric::PathGeometric::freeMemory(void)
{
    for (unsigned int i = 0 ; i < states.size() ; ++i)
        si_->freeState(states[i]);
}

double ompl::geometric::PathGeometric::length(void) const
{
    double L = 0.0;
    for (unsigned int i = 1 ; i < states.size() ; ++i)
        L += si_->distance(states[i-1], states[i]);
    return L;
}

bool ompl::geometric::PathGeometric::check(void) const
{
    bool result = true;
    if (states.size() > 0)
    {
        if (si_->isValid(states[0]))
        {
            int last = states.size() - 1;
            for (int j = 0 ; result && j < last ; ++j)
                if (!si_->checkMotion(states[j], states[j + 1]))
                    result = false;
        }
        else
            result = false;
    }

    return result;
}

void ompl::geometric::PathGeometric::print(std::ostream &out) const
{
    out << "Geometric path with " << states.size() << " states" << std::endl;
    for (unsigned int i = 0 ; i < states.size() ; ++i)
        si_->printState(states[i], out);
    out << std::endl;
}

bool ompl::geometric::PathGeometric::checkAndRepair(unsigned int attempts)
{
    if (states.empty())
        return true;
    if (states.size() == 1)
        return si_->isValid(states[0]);

    // a path with invalid endpoints cannot be fixed; planners should not return such paths anyway
    const int n1 = states.size() - 1;
    if (!si_->isValid(states[0]) || !si_->isValid(states[n1]))
        return false;

    base::State *temp = NULL;
    base::UniformValidStateSampler *uvss = NULL;
    bool result = true;

    for (int i = 1 ; i < n1 ; ++i)
        if (!si_->checkMotion(states[i-1], states[i]))
        {
            // we now compute a state around which to sample
            if (!temp)
                temp = si_->allocState();
            if (!uvss)
            {
                uvss = new base::UniformValidStateSampler(si_.get());
                uvss->setNrAttempts(attempts);
            }

            // and a radius of sampling around that state
            double radius = 0.0;

            if (si_->isValid(states[i]))
            {
                si_->copyState(temp, states[i]);
                radius = si_->distance(states[i-1], states[i]);
            }
            else
            {
                unsigned int nextValid = n1;
                for (int j = i + 1 ; j < n1 ; ++j)
                    if (si_->isValid(states[j]))
                    {
                        nextValid = j;
                        break;
                    }
                // we know nextValid will be initialised because n1 is certainly valid.
                si_->getStateManifold()->interpolate(states[i - 1], states[nextValid], 0.5, temp);
                radius = std::max(si_->distance(states[i-1], temp), si_->distance(states[i-1], states[i]));
            }

            bool success = false;

            for (unsigned int a = 0 ; a < attempts ; ++a)
                if (uvss->sampleNear(states[i], temp, radius))
                {
                    if (si_->checkMotion(states[i-1], states[i]))
                    {
                        success = true;
                        break;
                    }
                }
                else
                    break;
            if (!success)
            {
                result = false;
                break;
            }
        }

    // free potentially allocated memory
    if (temp)
        si_->freeState(temp);
    if (uvss)
        delete uvss;

    return result;
}

void ompl::geometric::PathGeometric::interpolate(unsigned int requestCount)
{
    if (requestCount < states.size() || states.size() < 2)
        return;

    unsigned int count = requestCount;

    // the remaining length of the path we need to add states along
    double remainingLength = length();

    // the new array of states this path will have
    std::vector<base::State*> newStates;
    const int n1 = states.size() - 1;

    for (int i = 0 ; i < n1 ; ++i)
    {
        base::State *s1 = states[i];
        base::State *s2 = states[i + 1];

        newStates.push_back(s1);

        // the maximum number of states that can be added on the current motion (without its endpoints)
        // such that we can at least fit the remaining states
        int maxNStates = count + i - states.size();

        if (maxNStates > 0)
        {
            // compute an approximate number of states the following segment needs to contain; this includes endpoints
            double segmentLength = si_->distance(s1, s2);
            int ns = i + 1 == n1 ? maxNStates + 2 : (int)floor(0.5 + (double)count * segmentLength / remainingLength) + 1;

            // if more than endpoints are needed
            if (ns > 2)
            {
                ns -= 2; // subtract endpoints

                // make sure we don't add too many states
                if (ns > maxNStates)
                    ns = maxNStates;

                // compute intermediate states
                std::vector<base::State*> block;
                unsigned int ans = si_->getMotionStates(s1, s2, block, ns, false, true);
                // sanity checks
                assert((int)ans == ns);
                assert(block.size() == ans);

                newStates.insert(newStates.end(), block.begin(), block.end());
            }
            else
                ns = 0;

            // update what remains to be done
            count -= (ns + 1);
            remainingLength -= segmentLength;
        }
        else
            count--;
    }

    // add the last state
    newStates.push_back(states[n1]);
    states.swap(newStates);
    if (requestCount != states.size())
        throw Exception("Internal error in path interpolation. This should never happen. Please contact the developers.");
}

void ompl::geometric::PathGeometric::reverse(void)
{
    std::reverse(states.begin(), states.end());
}

void ompl::geometric::PathGeometric::overlay(const PathGeometric &over, unsigned int startIndex)
{
    if (startIndex > states.size())
        throw Exception("Index of path is out of bounds");
    const base::StateManifoldPtr &sm = over.si_->getStateManifold();
    const base::StateManifoldPtr &dm = si_->getStateManifold();
    bool copy = !states.empty();
    for (unsigned int i = 0, j = startIndex ; i < over.states.size() ; ++i, ++j)
    {
        if (j == states.size())
        {
            base::State *s = si_->allocState();
            if (copy)
                si_->copyState(s, states.back());
            states.push_back(s);
        }

        __private_insertStateData(dm, states[j], sm, over.states[i]);
    }
}

void ompl::geometric::PathGeometric::append(const PathGeometric &path)
{
    if (path.si_->getStateManifold()->getName() == si_->getStateManifold()->getName())
    {
        PathGeometric copy(path);
        states.insert(states.end(), copy.states.begin(), copy.states.end());
        copy.states.clear();
    }
    else
    {
        const base::StateManifoldPtr &sm = path.si_->getStateManifold();
        const base::StateManifoldPtr &dm = si_->getStateManifold();
        bool copy = !states.empty();
        for (unsigned int i = 0 ; i < path.states.size() ; ++i)
        {
            base::State *s = si_->allocState();
            if (copy)
                si_->copyState(s, states.back());
            __private_insertStateData(dm, s, sm, path.states[i]);
            states.push_back(s);
        }
    }
}

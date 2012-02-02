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
#include <limits>
#include <boost/math/constants/constants.hpp>

ompl::geometric::PathGeometric::PathGeometric(const PathGeometric &path) : base::Path(path.si_)
{
    copyFrom(path);
}

ompl::geometric::PathGeometric::PathGeometric(const base::SpaceInformationPtr &si, const base::State *state) : base::Path(si)
{
    states.resize(1);
    states[0] = si_->cloneState(state);
}

ompl::geometric::PathGeometric::PathGeometric(const base::SpaceInformationPtr &si, const base::State *state1, const base::State *state2) : base::Path(si)
{
    states.resize(2);
    states[0] = si_->cloneState(state1);
    states[1] = si_->cloneState(state2);
}

ompl::geometric::PathGeometric& ompl::geometric::PathGeometric::operator=(const PathGeometric &other)
{
    if (this != &other)
    {
        freeMemory();
        si_ = other.si_;
        copyFrom(other);
    }
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

double ompl::geometric::PathGeometric::clearance(void) const
{
    double c = 0.0;
    for (unsigned int i = 0 ; i < states.size() ; ++i)
        c += si_->getStateValidityChecker()->clearance(states[i]);
    if (states.empty())
        c = std::numeric_limits<double>::infinity();
    else
        c /= (double)states.size();
    return c;
}

double ompl::geometric::PathGeometric::smoothness(void) const
{
    double s = 0.0;
    if (states.size() > 2)
    {
        double a = si_->distance(states[0], states[1]);
        for (unsigned int i = 2 ; i < states.size() ; ++i)
        {
            // view the path as a sequence of segments, and look at the triangles it forms:
            //          s1
            //          /\          s4
            //      a  /  \ b       |
            //        /    \        |
            //       /......\_______|
            //     s0    c   s2     s3
            //
            // use Pythagoras generalized theorem to find the cos of the angle between segments a and b
            double b = si_->distance(states[i-1], states[i]);
            double c = si_->distance(states[i-2], states[i]);
            double acosValue = (a*a + b*b - c*c) / (2.0*a*b);

            if (acosValue > -1.0 && acosValue < 1.0)
            {
                // the smoothness is actually the outside angle of the one we compute
                double angle = (boost::math::constants::pi<double>() - acos(acosValue));

                // and we normalize by the length of the segments
                double k = 2.0 * angle / (a + b);
                s += k * k;
            }
            a = b;
        }
    }
    return s;
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

std::pair<bool, bool> ompl::geometric::PathGeometric::checkAndRepair(unsigned int attempts)
{
    if (states.empty())
        return std::make_pair(true, true);
    if (states.size() == 1)
    {
        bool result = si_->isValid(states[0]);
        return std::make_pair(result, result);
    }

    // a path with invalid endpoints cannot be fixed; planners should not return such paths anyway
    const int n1 = states.size() - 1;
    if (!si_->isValid(states[0]) || !si_->isValid(states[n1]))
        return std::make_pair(false, false);

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
                si_->getStateSpace()->interpolate(states[i - 1], states[nextValid], 0.5, temp);
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
    bool originalValid = uvss == NULL;
    if (uvss)
        delete uvss;

    return std::make_pair(originalValid, result);
}

void ompl::geometric::PathGeometric::subdivide(void)
{
    if (states.size() < 2)
        return;
    std::vector<base::State*> newStates(1, states[0]);
    for (unsigned int i = 1 ; i < states.size() ; ++i)
    {
        base::State *temp = si_->allocState();
        si_->getStateSpace()->interpolate(newStates.back(), states[i], 0.5, temp);
        newStates.push_back(temp);
        newStates.push_back(states[i]);
    }
    states.swap(newStates);
}

void ompl::geometric::PathGeometric::interpolate(void)
{
    unsigned int n = 0;
    const int n1 = states.size() - 1;
    for (int i = 0 ; i < n1 ; ++i)
        n += si_->getStateSpace()->validSegmentCount(states[i], states[i + 1]);
    interpolate(n);
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
                if ((int)ans != ns || block.size() != ans)
                    throw Exception("Internal error in path interpolation. Incorrect number of intermediate states. Please contact the developers.");

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

void ompl::geometric::PathGeometric::random(void)
{
    freeMemory();
    states.resize(2);
    states[0] = si_->allocState();
    states[1] = si_->allocState();
    base::StateSamplerPtr ss = si_->allocStateSampler();
    ss->sampleUniform(states[0]);
    ss->sampleUniform(states[1]);
}

bool ompl::geometric::PathGeometric::randomValid(unsigned int attempts)
{
    freeMemory();
    states.resize(2);
    states[0] = si_->allocState();
    states[1] = si_->allocState();
    base::UniformValidStateSampler *uvss = new base::UniformValidStateSampler(si_.get());
    uvss->setNrAttempts(attempts);
    bool ok = false;
    for (unsigned int i = 0 ; i < attempts ; ++i)
    {
        if (uvss->sample(states[0]) && uvss->sample(states[1]))
            if (si_->checkMotion(states[0], states[1]))
            {
                ok = true;
                break;
            }
    }
    delete uvss;
    if (!ok)
    {
        freeMemory();
        states.clear();
    }
    return ok;
}

void ompl::geometric::PathGeometric::overlay(const PathGeometric &over, unsigned int startIndex)
{
    if (startIndex > states.size())
        throw Exception("Index on path is out of bounds");
    const base::StateSpacePtr &sm = over.si_->getStateSpace();
    const base::StateSpacePtr &dm = si_->getStateSpace();
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

        copyStateData(dm, states[j], sm, over.states[i]);
    }
}

void ompl::geometric::PathGeometric::append(const PathGeometric &path)
{
    if (path.si_->getStateSpace()->getName() == si_->getStateSpace()->getName())
    {
        PathGeometric copy(path);
        states.insert(states.end(), copy.states.begin(), copy.states.end());
        copy.states.clear();
    }
    else
        overlay(path, states.size());
}

void ompl::geometric::PathGeometric::keepAfter(const base::State *state)
{
    int index = getClosestIndex(state);
    if (index > 0)
    {
        if ((std::size_t)(index + 1) < states.size())
        {
            double b = si_->distance(state, states[index-1]);
            double a = si_->distance(state, states[index+1]);
            if (b > a)
                ++index;
        }
        for (int i = 0 ; i < index ; ++i)
            si_->freeState(states[i]);
        states.erase(states.begin(), states.begin() + index);
    }
}

void ompl::geometric::PathGeometric::keepBefore(const base::State *state)
{
    int index = getClosestIndex(state);
    if (index >= 0)
    {
        if (index > 0 && (std::size_t)(index + 1) < states.size())
        {
            double b = si_->distance(state, states[index-1]);
            double a = si_->distance(state, states[index+1]);
            if (b < a)
                --index;
        }
        if ((std::size_t)(index + 1) < states.size())
        {
            for (std::size_t i = index + 1 ; i < states.size() ; ++i)
                si_->freeState(states[i]);
            states.resize(index + 1);
        }
    }
}

int ompl::geometric::PathGeometric::getClosestIndex(const base::State *state) const
{
    if (states.empty())
        return -1;

    int index = 0;
    double min_d = si_->distance(states[0], state);
    for (std::size_t i = 1 ; i < states.size() ; ++i)
    {
        double d = si_->distance(states[i], state);
        if (d < min_d)
        {
            min_d = d;
            index = i;
        }
    }
    return index;
}

void ompl::geometric::PathGeometric::computeFastTimeParametrization(double maxVel, double maxAcc, std::vector<double> &times, unsigned int maxSteps)
{
    //  This implementation greately benefitted from discussions with Kenneth Anderson (http://sites.google.com/site/kennethaanderson/

    if (states.empty())
    {
        times.clear();
        return;
    }
    if (states.size() == 1)
    {
        times.resize(1);
        times[0] = 0.0;
        return;
    }
    if (states.size() == 2)
    {
        double d = si_->distance(states[0], states[1]);
        times.resize(2);
        times[0] = 0.0;
        times[1] = std::max(2.0 * d / maxVel, sqrt(2.0 * d / maxAcc));
        return;
    }

    times.resize(states.size());
    times[0] = 0.0;
    std::vector<double> vel(states.size(), maxVel);
    vel.front() = vel.back() = 0.0;
    std::vector<double> L(states.size() - 1);
    for (std::size_t i = 0 ; i < L.size() ; ++i)
        L[i] = si_->distance(states[i], states[i + 1]);

    static const double velFactor = 0.95;

    bool change = true;
    unsigned int steps = 0;
    while (change && steps < maxSteps)
    {
        ++steps;
        for (std::size_t i = 1 ; i < times.size() ; ++i)
            times[i] = times[i - 1] + (2.0 * L[i-1]) / (vel[i-1] + vel[i]);

        change = false;
        for (std::size_t i = 0 ; i < L.size() ; ++i)
        {
            double acc = (vel[i + 1] - vel[i]) / (times[i + 1] - times[i]);
            if (acc > maxAcc)
            {
                vel[i + 1] *= velFactor;
                change = true;
            }
            else
                if (acc < -maxAcc)
                {
                    vel[i] *= velFactor;
                    change = true;
                }
        }

        if (change)
            for (int i = L.size() - 1 ; i >= 0 ; --i)
            {
                double acc = (vel[i + 1] - vel[i]) / (times[i + 1] - times[i]);
                if (acc > maxAcc)
                {
                    vel[i + 1] *= velFactor;
                    change = true;
                }
                else
                    if (acc < -maxAcc)
                    {
                        vel[i] *= velFactor;
                        change = true;
                    }
            }
    }

}

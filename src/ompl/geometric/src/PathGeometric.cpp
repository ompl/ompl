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
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/ScopedState.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <boost/math/constants/constants.hpp>

ompl::geometric::PathGeometric::PathGeometric(const PathGeometric &path) : base::Path(path.si_)
{
    copyFrom(path);
}

ompl::geometric::PathGeometric::PathGeometric(const base::SpaceInformationPtr &si, const base::State *state)
  : base::Path(si)
{
    states_.resize(1);
    states_[0] = si_->cloneState(state);
}

ompl::geometric::PathGeometric::PathGeometric(const base::SpaceInformationPtr &si, const base::State *state1,
                                              const base::State *state2)
  : base::Path(si)
{
    states_.resize(2);
    states_[0] = si_->cloneState(state1);
    states_[1] = si_->cloneState(state2);
}

ompl::geometric::PathGeometric &ompl::geometric::PathGeometric::operator=(const PathGeometric &other)
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
    states_.resize(other.states_.size());
    for (unsigned int i = 0; i < states_.size(); ++i)
        states_[i] = si_->cloneState(other.states_[i]);
}

void ompl::geometric::PathGeometric::freeMemory()
{
    for (auto &state : states_)
        si_->freeState(state);
}

ompl::base::Cost ompl::geometric::PathGeometric::cost(const base::OptimizationObjectivePtr &opt) const
{
    if (states_.empty())
        return opt->identityCost();
    // Compute path cost by accumulating the cost along the path
    base::Cost cost(opt->initialCost(states_.front()));
    for (std::size_t i = 1; i < states_.size(); ++i)
        cost = opt->combineCosts(cost, opt->motionCost(states_[i - 1], states_[i]));
    cost = opt->combineCosts(cost, opt->terminalCost(states_.back()));
    return cost;
}

double ompl::geometric::PathGeometric::length() const
{
    double L = 0.0;
    for (unsigned int i = 1; i < states_.size(); ++i)
        L += si_->distance(states_[i - 1], states_[i]);
    return L;
}

double ompl::geometric::PathGeometric::clearance() const
{
    double c = 0.0;
    for (auto state : states_)
        c += si_->getStateValidityChecker()->clearance(state);
    if (states_.empty())
        c = std::numeric_limits<double>::infinity();
    else
        c /= (double)states_.size();
    return c;
}

double ompl::geometric::PathGeometric::smoothness() const
{
    double s = 0.0;
    if (states_.size() > 2)
    {
        double a = si_->distance(states_[0], states_[1]);
        for (unsigned int i = 2; i < states_.size(); ++i)
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
            double b = si_->distance(states_[i - 1], states_[i]);
            double c = si_->distance(states_[i - 2], states_[i]);
            double acosValue = (a * a + b * b - c * c) / (2.0 * a * b);

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

bool ompl::geometric::PathGeometric::check() const
{
    // make sure state validity checker is set
    if (!si_->isSetup())
        si_->setup();

    bool result = true;
    if (states_.size() > 0)
    {
        if (si_->isValid(states_[0]))
        {
            int last = states_.size() - 1;
            for (int j = 0; result && j < last; ++j)
                if (!si_->checkMotion(states_[j], states_[j + 1]))
                    result = false;
        }
        else
            result = false;
    }

    return result;
}

void ompl::geometric::PathGeometric::print(std::ostream &out) const
{
    out << "Geometric path with " << states_.size() << " states" << std::endl;
    for (auto state : states_)
        si_->printState(state, out);
    out << std::endl;
}
void ompl::geometric::PathGeometric::printAsMatrix(std::ostream &out) const
{
    const base::StateSpace *space(si_->getStateSpace().get());
    std::vector<double> reals;
    for (auto state : states_)
    {
        space->copyToReals(reals, state);
        std::copy(reals.begin(), reals.end(), std::ostream_iterator<double>(out, " "));
        out << std::endl;
    }
    out << std::endl;
}

std::pair<bool, bool> ompl::geometric::PathGeometric::checkAndRepair(unsigned int attempts)
{
    if (states_.empty())
        return std::make_pair(true, true);
    if (states_.size() == 1)
    {
        bool result = si_->isValid(states_[0]);
        return std::make_pair(result, result);
    }

    // a path with invalid endpoints cannot be fixed; planners should not return such paths anyway
    const int n1 = states_.size() - 1;
    if (!si_->isValid(states_[0]) || !si_->isValid(states_[n1]))
        return std::make_pair(false, false);

    base::State *temp = nullptr;
    base::UniformValidStateSampler *uvss = nullptr;
    bool result = true;

    for (int i = 1; i < n1; ++i)
        if (!si_->checkMotion(states_[i - 1], states_[i]) ||
            // the penultimate state in the path needs an additional check:
            // the motion between that state and the last state needs to be
            // valid as well since we cannot change the last state.
            (i == n1 - 1 && !si_->checkMotion(states_[i], states_[i + 1])))
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

            if (si_->isValid(states_[i]))
            {
                si_->copyState(temp, states_[i]);
                radius = si_->distance(states_[i - 1], states_[i]);
            }
            else
            {
                unsigned int nextValid = n1 - 1;
                for (int j = i + 1; j < n1; ++j)
                    if (si_->isValid(states_[j]))
                    {
                        nextValid = j;
                        break;
                    }
                // we know nextValid will be initialised because n1 - 1 is certainly valid.
                si_->getStateSpace()->interpolate(states_[i - 1], states_[nextValid], 0.5, temp);
                radius = std::max(si_->distance(states_[i - 1], temp), si_->distance(states_[i - 1], states_[i]));
            }

            bool success = false;

            for (unsigned int a = 0; a < attempts; ++a)
                if (uvss->sampleNear(states_[i], temp, radius))
                {
                    if (si_->checkMotion(states_[i - 1], states_[i]) &&
                        // the penultimate state needs an additional check
                        // (see comment at the top of outermost for-loop)
                        (i < n1 - 1 || si_->checkMotion(states_[i], states_[i + 1])))
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
    bool originalValid = uvss == nullptr;
    if (uvss)
        delete uvss;

    return std::make_pair(originalValid, result);
}

void ompl::geometric::PathGeometric::subdivide()
{
    if (states_.size() < 2)
        return;
    std::vector<base::State *> newStates(1, states_[0]);
    for (unsigned int i = 1; i < states_.size(); ++i)
    {
        base::State *temp = si_->allocState();
        si_->getStateSpace()->interpolate(newStates.back(), states_[i], 0.5, temp);
        newStates.push_back(temp);
        newStates.push_back(states_[i]);
    }
    states_.swap(newStates);
}

void ompl::geometric::PathGeometric::interpolate()
{
    std::vector<base::State *> newStates;
    const int segments = states_.size() - 1;

    for (int i = 0; i < segments; ++i)
    {
        base::State *s1 = states_[i];
        base::State *s2 = states_[i + 1];

        newStates.push_back(s1);
        unsigned int n = si_->getStateSpace()->validSegmentCount(s1, s2);

        std::vector<base::State *> block;
        si_->getMotionStates(s1, s2, block, n - 1, false, true);
        newStates.insert(newStates.end(), block.begin(), block.end());
    }
    newStates.push_back(states_[segments]);
    states_.swap(newStates);
}

void ompl::geometric::PathGeometric::interpolate(unsigned int requestCount)
{
    if (requestCount < states_.size() || states_.size() < 2)
        return;

    unsigned int count = requestCount;

    // the remaining length of the path we need to add states along
    double remainingLength = length();

    // the new array of states this path will have
    std::vector<base::State *> newStates;
    const int n1 = states_.size() - 1;

    for (int i = 0; i < n1; ++i)
    {
        base::State *s1 = states_[i];
        base::State *s2 = states_[i + 1];

        newStates.push_back(s1);

        // the maximum number of states that can be added on the current motion (without its endpoints)
        // such that we can at least fit the remaining states
        int maxNStates = count + i - states_.size();

        if (maxNStates > 0)
        {
            // compute an approximate number of states the following segment needs to contain; this includes endpoints
            double segmentLength = si_->distance(s1, s2);
            int ns =
                i + 1 == n1 ? maxNStates + 2 : (int)floor(0.5 + (double)count * segmentLength / remainingLength) + 1;

            // if more than endpoints are needed
            if (ns > 2)
            {
                ns -= 2;  // subtract endpoints

                // make sure we don't add too many states
                if (ns > maxNStates)
                    ns = maxNStates;

                // compute intermediate states
                std::vector<base::State *> block;
                si_->getMotionStates(s1, s2, block, ns, false, true);
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
    newStates.push_back(states_[n1]);
    states_.swap(newStates);
}

void ompl::geometric::PathGeometric::reverse()
{
    std::reverse(states_.begin(), states_.end());
}

void ompl::geometric::PathGeometric::random()
{
    freeMemory();
    states_.resize(2);
    states_[0] = si_->allocState();
    states_[1] = si_->allocState();
    base::StateSamplerPtr ss = si_->allocStateSampler();
    ss->sampleUniform(states_[0]);
    ss->sampleUniform(states_[1]);
}

bool ompl::geometric::PathGeometric::randomValid(unsigned int attempts)
{
    freeMemory();
    states_.resize(2);
    states_[0] = si_->allocState();
    states_[1] = si_->allocState();
    base::UniformValidStateSampler uvss(si_.get());
    uvss.setNrAttempts(attempts);
    bool ok = false;
    for (unsigned int i = 0; i < attempts; ++i)
    {
        if (uvss.sample(states_[0]) && uvss.sample(states_[1]))
            if (si_->checkMotion(states_[0], states_[1]))
            {
                ok = true;
                break;
            }
    }
    if (!ok)
    {
        freeMemory();
        states_.clear();
    }
    return ok;
}

void ompl::geometric::PathGeometric::overlay(const PathGeometric &over, unsigned int startIndex)
{
    if (startIndex > states_.size())
        throw Exception("Index on path is out of bounds");
    const base::StateSpacePtr &sm = over.si_->getStateSpace();
    const base::StateSpacePtr &dm = si_->getStateSpace();
    bool copy = !states_.empty();
    for (unsigned int i = 0, j = startIndex; i < over.states_.size(); ++i, ++j)
    {
        if (j == states_.size())
        {
            base::State *s = si_->allocState();
            if (copy)
                si_->copyState(s, states_.back());
            states_.push_back(s);
        }

        copyStateData(dm, states_[j], sm, over.states_[i]);
    }
}

void ompl::geometric::PathGeometric::append(const base::State *state)
{
    states_.push_back(si_->cloneState(state));
}

void ompl::geometric::PathGeometric::append(const PathGeometric &path)
{
    if (path.si_->getStateSpace()->getName() == si_->getStateSpace()->getName())
    {
        PathGeometric copy(path);
        states_.insert(states_.end(), copy.states_.begin(), copy.states_.end());
        copy.states_.clear();
    }
    else
        overlay(path, states_.size());
}

void ompl::geometric::PathGeometric::prepend(const base::State *state)
{
    states_.insert(states_.begin(), si_->cloneState(state));
}

void ompl::geometric::PathGeometric::keepAfter(const base::State *state)
{
    int index = getClosestIndex(state);
    if (index > 0)
    {
        if ((std::size_t)(index + 1) < states_.size())
        {
            double b = si_->distance(state, states_[index - 1]);
            double a = si_->distance(state, states_[index + 1]);
            if (b > a)
                ++index;
        }
        for (int i = 0; i < index; ++i)
            si_->freeState(states_[i]);
        states_.erase(states_.begin(), states_.begin() + index);
    }
}

void ompl::geometric::PathGeometric::keepBefore(const base::State *state)
{
    int index = getClosestIndex(state);
    if (index >= 0)
    {
        if (index > 0 && (std::size_t)(index + 1) < states_.size())
        {
            double b = si_->distance(state, states_[index - 1]);
            double a = si_->distance(state, states_[index + 1]);
            if (b < a)
                --index;
        }
        if ((std::size_t)(index + 1) < states_.size())
        {
            for (std::size_t i = index + 1; i < states_.size(); ++i)
                si_->freeState(states_[i]);
            states_.resize(index + 1);
        }
    }
}

int ompl::geometric::PathGeometric::getClosestIndex(const base::State *state) const
{
    if (states_.empty())
        return -1;

    int index = 0;
    double min_d = si_->distance(states_[0], state);
    for (std::size_t i = 1; i < states_.size(); ++i)
    {
        double d = si_->distance(states_[i], state);
        if (d < min_d)
        {
            min_d = d;
            index = i;
        }
    }
    return index;
}

void ompl::geometric::PathGeometric::clear()
{
    freeMemory();
    states_.clear();
}

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University, Inc.
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

#include "ompl/geometric/PathSimplifier.h"
#include "ompl/tools/config/MagicConstants.h"
#include <algorithm>
#include <limits>
#include <cstdlib>
#include <cmath>
#include <map>

bool ompl::geometric::PathSimplifier::freeStates() const
{
    return freeStates_;
}

void ompl::geometric::PathSimplifier::freeStates(bool flag)
{
    freeStates_ = flag;
}

/* Based on COMP450 2010 project of Yun Yu and Linda Hill (Rice University) */
void ompl::geometric::PathSimplifier::smoothBSpline(PathGeometric &path, unsigned int maxSteps, double minChange)
{
    if (path.getStateCount() < 3)
        return;

    const base::SpaceInformationPtr &si = path.getSpaceInformation();
    std::vector<base::State*> &states = path.getStates();

    base::State *temp1 = si->allocState();
    base::State *temp2 = si->allocState();

    for (unsigned int s = 0 ; s < maxSteps ; ++s)
    {
        path.subdivide();

        unsigned int i = 2, u = 0, n1 = states.size() - 1;
        while (i < n1)
        {
            if (si->isValid(states[i - 1]))
            {
                si->getStateSpace()->interpolate(states[i - 1], states[i], 0.5, temp1);
                si->getStateSpace()->interpolate(states[i], states[i + 1], 0.5, temp2);
                si->getStateSpace()->interpolate(temp1, temp2, 0.5, temp1);
                if (si->checkMotion(states[i - 1], temp1) && si->checkMotion(temp1, states[i + 1]))
                {
                    if (si->distance(states[i], temp1) > minChange)
                    {
                        si->copyState(states[i], temp1);
                        ++u;
                    }
                }
            }

            i += 2;
        }

        if (u == 0)
            break;
    }

    si->freeState(temp1);
    si->freeState(temp2);
}

bool ompl::geometric::PathSimplifier::reduceVertices(PathGeometric &path, unsigned int maxSteps, unsigned int maxEmptySteps, double rangeRatio)
{
    if (path.getStateCount() < 3)
        return false;

    if (maxSteps == 0)
        maxSteps = path.getStateCount();

    if (maxEmptySteps == 0)
        maxEmptySteps = path.getStateCount();

    bool result = false;
    unsigned int nochange = 0;
    const base::SpaceInformationPtr &si = path.getSpaceInformation();
    std::vector<base::State*> &states = path.getStates();

    if (si->checkMotion(states.front(), states.back()))
    {
        if (freeStates_)
            for (std::size_t i = 2 ; i < states.size() ; ++i)
                si->freeState(states[i-1]);
        std::vector<base::State*> newStates(2);
        newStates[0] = states.front();
        newStates[1] = states.back();
        states.swap(newStates);
        result = true;
    }
    else
        for (unsigned int i = 0 ; i < maxSteps && nochange < maxEmptySteps ; ++i, ++nochange)
        {
            int count = states.size();
            int maxN  = count - 1;
            int range = 1 + (int)(floor(0.5 + (double)count * rangeRatio));

            int p1 = rng_.uniformInt(0, maxN);
            int p2 = rng_.uniformInt(std::max(p1 - range, 0), std::min(maxN, p1 + range));
            if (abs(p1 - p2) < 2)
            {
                if (p1 < maxN - 1)
                    p2 = p1 + 2;
                else
                    if (p1 > 1)
                        p2 = p1 - 2;
                    else
                        continue;
            }

            if (p1 > p2)
                std::swap(p1, p2);

            if (si->checkMotion(states[p1], states[p2]))
            {
                if (freeStates_)
                    for (int j = p1 + 1 ; j < p2 ; ++j)
                        si->freeState(states[j]);
                states.erase(states.begin() + p1 + 1, states.begin() + p2);
                nochange = 0;
                result = true;
            }
        }
    return result;
}

bool ompl::geometric::PathSimplifier::shortcutPath(PathGeometric &path, unsigned int maxSteps, unsigned int maxEmptySteps, double rangeRatio, double snapToVertex)
{
    if (path.getStateCount() < 3)
        return false;

    if (maxSteps == 0)
        maxSteps = path.getStateCount();

    if (maxEmptySteps == 0)
        maxEmptySteps = path.getStateCount();

    const base::SpaceInformationPtr &si = path.getSpaceInformation();
    std::vector<base::State*> &states = path.getStates();

    std::vector<double> dists(states.size(), 0.0);
    for (unsigned int i = 1 ; i < dists.size() ; ++i)
        dists[i] = dists[i - 1] + si->distance(states[i-1], states[i]);
    double threshold = dists.back() * snapToVertex;
    double rd = rangeRatio * dists.back();

    base::State *temp0 = si->allocState();
    base::State *temp1 = si->allocState();
    bool result = false;
    unsigned int nochange = 0;
    for (unsigned int i = 0 ; i < maxSteps && nochange < maxEmptySteps ; ++i, ++nochange)
    {
        base::State *s0 = NULL;
        int index0 = -1;
        double t0 = 0.0;
        double p0 = rng_.uniformReal(0.0, dists.back());
        std::vector<double>::iterator pit = std::lower_bound(dists.begin(), dists.end(), p0);
        int pos0 = pit == dists.end() ? dists.size() - 1 : pit - dists.begin();

        if (pos0 == 0 || dists[pos0] - p0 < threshold)
            index0 = pos0;
        else
        {
            while (pos0 > 0 && p0 < dists[pos0])
                --pos0;
            if (p0 - dists[pos0] < threshold)
                index0 = pos0;
        }

        base::State *s1 = NULL;
        int index1 = -1;
        double t1 = 0.0;
        double p1 = rng_.uniformReal(std::max(0.0, p0 - rd), std::min(p0 + rd, dists.back()));
        pit = std::lower_bound(dists.begin(), dists.end(), p1);
        int pos1 = pit == dists.end() ? dists.size() - 1 : pit - dists.begin();

        if (pos1 == 0 || dists[pos1] - p1 < threshold)
            index1 = pos1;
        else
        {
            while (pos1 > 0 && p1 < dists[pos1])
                --pos1;
            if (p1 - dists[pos1] < threshold)
                index1 = pos1;
        }

        if (pos0 == pos1 || index0 == pos1 || index1 == pos0 ||
            pos0 + 1 == index1 || pos1 + 1 == index0 ||
            (index0 >=0 && index1 >= 0 && abs(index0 - index1) < 2))
            continue;

        if (index0 >= 0)
            s0 = states[index0];
        else
        {
            t0 = (p0 - dists[pos0]) / (dists[pos0 + 1] - dists[pos0]);
            si->getStateSpace()->interpolate(states[pos0], states[pos0 + 1], t0, temp0);
            s0 = temp0;
        }

        if (index1 >= 0)
            s1 = states[index1];
        else
        {
            t1 = (p1 - dists[pos1]) / (dists[pos1 + 1] - dists[pos1]);
            si->getStateSpace()->interpolate(states[pos1], states[pos1 + 1], t1, temp1);
            s1 = temp1;
        }

        if (si->checkMotion(s0, s1))
        {
            if (pos0 > pos1)
            {
                std::swap(pos0, pos1);
                std::swap(index0, index1);
                std::swap(s0, s1);
                std::swap(t0, t1);
            }

            if (index0 < 0 && index1 < 0)
            {
                if (pos0 + 1 == pos1)
                {
                    si->copyState(states[pos1], s0);
                    states.insert(states.begin() + pos1 + 1, si->cloneState(s1));
                }
                else
                {
                    if (freeStates_)
                        for (int j = pos0 + 2 ; j < pos1 ; ++j)
                            si->freeState(states[j]);
                    si->copyState(states[pos0 + 1], s0);
                    si->copyState(states[pos1], s1);
                    states.erase(states.begin() + pos0 + 2, states.begin() + pos1);
                }
            }
            else
                if (index0 >= 0 && index1 >= 0)
                {
                    if (freeStates_)
                        for (int j = index0 + 1 ; j < index1 ; ++j)
                            si->freeState(states[j]);
                    states.erase(states.begin() + index0 + 1, states.begin() + index1);
                }
                else
                    if (index0 < 0 && index1 >= 0)
                    {
                        if (freeStates_)
                            for (int j = pos0 + 2 ; j < index1 ; ++j)
                                si->freeState(states[j]);
                        si->copyState(states[pos0 + 1], s0);
                        states.erase(states.begin() + pos0 + 2, states.begin() + index1);
                    }
                    else
                        if (index0 >= 0 && index1 < 0)
                        {
                            if (freeStates_)
                                for (int j = index0 + 1 ; j < pos1 ; ++j)
                                    si->freeState(states[j]);
                            si->copyState(states[pos1], s1);
                            states.erase(states.begin() + index0 + 1, states.begin() + pos1);
                        }

            // fix the helper variables
            dists.resize(states.size(), 0.0);
            for (unsigned int j = pos0 + 1 ; j < dists.size() ; ++j)
                dists[j] = dists[j - 1] + si->distance(states[j-1], states[j]);
            threshold = dists.back() * snapToVertex;
            rd = rangeRatio * dists.back();
            result = true;
            nochange = 0;
        }
    }

    si->freeState(temp1);
    si->freeState(temp0);
    return result;
}

bool ompl::geometric::PathSimplifier::collapseCloseVertices(PathGeometric &path, unsigned int maxSteps, unsigned int maxEmptySteps)
{
    if (path.getStateCount() < 3)
        return false;

    if (maxSteps == 0)
        maxSteps = path.getStateCount();

    if (maxEmptySteps == 0)
        maxEmptySteps = path.getStateCount();

    const base::SpaceInformationPtr &si = path.getSpaceInformation();
    std::vector<base::State*> &states = path.getStates();

    // compute pair-wise distances in path (construct only half the matrix)
    std::map<std::pair<const base::State*, const base::State*>, double> distances;
    for (unsigned int i = 0 ; i < states.size() ; ++i)
        for (unsigned int j = i + 2 ; j < states.size() ; ++j)
            distances[std::make_pair(states[i], states[j])] = si->distance(states[i], states[j]);

    bool result = false;
    unsigned int nochange = 0;
    for (unsigned int s = 0 ; s < maxSteps && nochange < maxEmptySteps ; ++s, ++nochange)
    {
        // find closest pair of points
        double minDist = std::numeric_limits<double>::infinity();
        int p1 = -1;
        int p2 = -1;
        for (unsigned int i = 0 ; i < states.size() ; ++i)
            for (unsigned int j = i + 2 ; j < states.size() ; ++j)
            {
                double d = distances[std::make_pair(states[i], states[j])];
                if (d < minDist)
                {
                    minDist = d;
                    p1 = i;
                    p2 = j;
                }
            }

        if (p1 >= 0 && p2 >= 0)
        {
            if (si->checkMotion(states[p1], states[p2]))
            {
                if (freeStates_)
                    for (int i = p1 + 1 ; i < p2 ; ++i)
                        si->freeState(states[i]);
                states.erase(states.begin() + p1 + 1, states.begin() + p2);
                result = true;
                nochange = 0;
            }
            else
                distances[std::make_pair(states[p1], states[p2])] = std::numeric_limits<double>::infinity();
        }
        else
            break;
    }
    return result;
}

void ompl::geometric::PathSimplifier::simplifyMax(PathGeometric &path)
{
    reduceVertices(path);
    collapseCloseVertices(path);
    // BSpline and checkAndRepair code only makes sense in a metric space.
    if (si_->getStateSpace()->isMetricSpace())
    {
        smoothBSpline(path, 4, path.length()/100.0);
        const std::pair<bool, bool> &p = path.checkAndRepair(magic::MAX_VALID_SAMPLE_ATTEMPTS);
        if (!p.second)
            OMPL_WARN("Solution path may slightly touch on an invalid region of the state space");
        else
            if (!p.first)
                OMPL_DEBUG("The solution path was slightly touching on an invalid region of the state space, but it was successfully fixed.");
    }
}

void ompl::geometric::PathSimplifier::simplify(PathGeometric &path, double maxTime)
{
    simplify(path, base::timedPlannerTerminationCondition(maxTime));
}

void ompl::geometric::PathSimplifier::simplify(PathGeometric &path, const base::PlannerTerminationCondition &ptc)
{
    if (path.getStateCount() < 3)
        return;

    // try a randomized step of connecting vertices
    bool tryMore = false;
    if (ptc == false)
        tryMore = reduceVertices(path);

    // try to collapse close-by vertices
    if (ptc == false)
        collapseCloseVertices(path);

    // try to reduce verices some more, if there is any point in doing so
    int times = 0;
    while (tryMore && ptc == false && ++times <= 5)
        tryMore = reduceVertices(path);

    // if the space is metric, we can do some additional smoothing
    if(si_->getStateSpace()->isMetricSpace())
    {
        // run a more complex short-cut algorithm : allow splitting path segments
        if (ptc == false)
            tryMore = shortcutPath(path);
        else
            tryMore = false;

        // run the short-cut algorithm some more, if it makes a difference
        times = 0;
        while (tryMore && ptc == false && ++times <= 5)
            tryMore = shortcutPath(path);

        // smooth the path with BSpline interpolation
        if(ptc == false)
            smoothBSpline(path, 3, path.length()/100.0);

        // we always run this if the metric-space algorithms were run.  In non-metric spaces this does not work.
        const std::pair<bool, bool> &p = path.checkAndRepair(magic::MAX_VALID_SAMPLE_ATTEMPTS);
        if (!p.second)
            OMPL_WARN("Solution path may slightly touch on an invalid region of the state space");
        else
            if (!p.first)
                OMPL_DEBUG("The solution path was slightly touching on an invalid region of the state space, but it was successfully fixed.");
    }
}

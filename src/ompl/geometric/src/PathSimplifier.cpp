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

/* Based on COMP450 2010 project of Yun Yu and Linda Hill (Rice University) */
void ompl::geometric::PathSimplifier::smoothBSpline(PathGeometric &path, unsigned int maxSteps, double minChange)
{
    if (path.states.size() < 3)
        return;

    const base::SpaceInformationPtr &si = path.getSpaceInformation();
    base::State *temp1 = si->allocState();
    base::State *temp2 = si->allocState();

    for (unsigned int s = 0 ; s < maxSteps ; ++s)
    {
        path.subdivide();

        unsigned int i = 2, u = 0, n1 = path.states.size() - 1;
        while (i < n1)
        {
            if (si->isValid(path.states[i - 1]))
            {
                si->getStateSpace()->interpolate(path.states[i - 1], path.states[i], 0.5, temp1);
                si->getStateSpace()->interpolate(path.states[i], path.states[i + 1], 0.5, temp2);
                si->getStateSpace()->interpolate(temp1, temp2, 0.5, temp1);
                if (si->checkMotion(path.states[i - 1], temp1) && si->checkMotion(temp1, path.states[i + 1]))
                {
                    if (si->distance(path.states[i], temp1) > minChange)
                    {
                        si->copyState(path.states[i], temp1);
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

void ompl::geometric::PathSimplifier::reduceVertices(PathGeometric &path, unsigned int maxSteps, unsigned int maxEmptySteps, double rangeRatio)
{
    if (path.states.size() < 3)
        return;

    if (maxSteps == 0)
        maxSteps = path.states.size();

    if (maxEmptySteps == 0)
        maxEmptySteps = path.states.size();

    unsigned int nochange = 0;
    const base::SpaceInformationPtr &si = path.getSpaceInformation();

    for (unsigned int i = 0 ; i < maxSteps && nochange < maxEmptySteps ; ++i, ++nochange)
    {
        int count = path.states.size();
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

        if (si->checkMotion(path.states[p1], path.states[p2]))
        {
            for (int j = p1 + 1 ; j < p2 ; ++j)
                si->freeState(path.states[j]);
            path.states.erase(path.states.begin() + p1 + 1, path.states.begin() + p2);
            nochange = 0;
        }
    }
}

void ompl::geometric::PathSimplifier::shortcutPath(PathGeometric &path, unsigned int maxSteps, unsigned int maxEmptySteps, double rangeRatio, double snapToVertex)
{
    if (path.states.size() < 3)
        return;

    if (maxSteps == 0)
        maxSteps = path.states.size();

    if (maxEmptySteps == 0)
        maxEmptySteps = path.states.size();

    const base::SpaceInformationPtr &si = path.getSpaceInformation();

    std::vector<double> dists(path.states.size(), 0.0);
    for (unsigned int i = 1 ; i < dists.size() ; ++i)
        dists[i] = dists[i - 1] + si->distance(path.states[i-1], path.states[i]);
    double threshold = dists.back() * snapToVertex;
    double rd = rangeRatio * dists.back();

    base::State *temp0 = si->allocState();
    base::State *temp1 = si->allocState();

    unsigned int nochange = 0;
    for (unsigned int i = 0 ; i < maxSteps && nochange < maxEmptySteps ; ++i, ++nochange)
    {
        base::State *s0 = NULL;
        int index0 = -1;
        double t0 = 0.0;
        double p0 = rng_.uniformReal(0.0, dists.back());
        std::vector<double>::iterator pit = std::lower_bound(dists.begin(), dists.end(), p0);
        int pos0 = pit == dists.end() ? dists.size() - 1 : pit - dists.begin();
        std::cout << "sampled p0 = " << p0 << ", got pos0 = " << pos0 << " dists[pos0] = " << dists[pos0] << std::endl;

        if (pos0 == 0 || dists[pos0] - p0 < threshold)
            index0 = pos0;
        else
        {
            while (pos0 > 0 && p0 > dists[pos0])
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
        std::cout << "sampled p1 = " << p1 << ", got pos1 = " << pos1 << " dists[pos1] = " << dists[pos1] << std::endl;

        if (pos1 == 0 || dists[pos1] - p1 < threshold)
            index1 = pos1;
        else
        {
            while (pos1 > 0 && p1 > dists[pos1])
                --pos1;
            if (p1 - dists[pos1] < threshold)
                index1 = pos1;
        }

        if (pos0 == pos1 || index0 == pos1 || index1 == pos0 ||
            pos0 + 1 == index1 || pos1 + 1 == index0 ||
            (index0 >=0 && index1 >= 0 && abs(index0 - index1) < 2))
            continue;

        if (index0 >= 0)
            s0 = path.states[index0];
        else
        {
            t0 = (p0 - dists[pos0]) / (dists[pos0 + 1] - dists[pos0]);
            std::cout << "interpolating at t0 = " << t0 << std::endl;
            si->getStateSpace()->interpolate(path.states[pos0], path.states[pos0 + 1], t0, temp0);
            s0 = temp0;
        }

        if (index1 >= 0)
            s1 = path.states[index1];
        else
        {
            t1 = (p1 - dists[pos1]) / (dists[pos1 + 1] - dists[pos1]);
            std::cout << "interpolating at t1 = " << t1 << std::endl;
            si->getStateSpace()->interpolate(path.states[pos1], path.states[pos1 + 1], t1, temp1);
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
                    std::cout << "case X X a\n";
                    si->copyState(path.states[pos1], s0);
                    path.states.insert(path.states.begin() + pos1 + 1, si->cloneState(s1));
                }
                else
                {
                    std::cout << "case X X b\n";
                    for (unsigned int j = pos0 + 2 ; j < pos1 ; ++j)
                        si->freeState(path.states[j]);
                    si->copyState(path.states[pos0 + 1], s0);
                    si->copyState(path.states[pos1], s1);
                    path.states.erase(path.states.begin() + pos0 + 2, path.states.begin() + pos1);
                }
            }
            else
                if (index0 >= 0 && index1 >= 0)
                {
                    std::cout << "case 0 0\n";
                    for (unsigned int j = index0 + 1 ; j < index1 ; ++j)
                        si->freeState(path.states[j]);
                    path.states.erase(path.states.begin() + index0 + 1, path.states.begin() + index1);
                }
                else
                    if (index0 < 0 && index1 >= 0)
                    {
                        std::cout << "case X 0\n";
                        for (unsigned int j = pos0 + 2 ; j < index1 ; ++j)
                            si->freeState(path.states[j]);
                        si->copyState(path.states[pos0 + 1], s0);
                        path.states.erase(path.states.begin() + pos0 + 2, path.states.begin() + index1);
                    }
                    else
                        if (index0 >= 0 && index1 < 0)
                        {
                            std::cout << "case 0 X\n";
                            for (unsigned int j = index0 + 1 ; j < pos1 ; ++j)
                                si->freeState(path.states[j]);
                            si->copyState(path.states[pos1], s1);
                            path.states.erase(path.states.begin() + index0 + 1, path.states.begin() + pos1);
                        }

            // fix the helper variables
            dists.resize(path.states.size(), 0.0);
            for (unsigned int j = pos0 + 1 ; j < dists.size() ; ++j)
                dists[j] = dists[j - 1] + si->distance(path.states[j-1], path.states[j]);
            threshold = dists.back() * snapToVertex;
            rd = rangeRatio * dists.back();

            nochange = 0;
        }
    }

    si->freeState(temp1);
    si->freeState(temp0);
}

void ompl::geometric::PathSimplifier::collapseCloseVertices(PathGeometric &path, unsigned int maxSteps, unsigned int maxEmptySteps)
{
    if (path.states.size() < 3)
        return;

    if (maxSteps == 0)
        maxSteps = path.states.size();

    if (maxEmptySteps == 0)
        maxEmptySteps = path.states.size();

    const base::SpaceInformationPtr &si = path.getSpaceInformation();

    // compute pair-wise distances in path (construct only half the matrix)
    std::map<std::pair<const base::State*, const base::State*>, double> distances;
    for (unsigned int i = 0 ; i < path.states.size() ; ++i)
        for (unsigned int j = i + 2 ; j < path.states.size() ; ++j)
            distances[std::make_pair(path.states[i], path.states[j])] = si->distance(path.states[i], path.states[j]);

    unsigned int nochange = 0;

    for (unsigned int s = 0 ; s < maxSteps && nochange < maxEmptySteps ; ++s, ++nochange)
    {
        // find closest pair of points
        double minDist = std::numeric_limits<double>::infinity();
        int p1 = -1;
        int p2 = -1;
        for (unsigned int i = 0 ; i < path.states.size() ; ++i)
            for (unsigned int j = i + 2 ; j < path.states.size() ; ++j)
            {
                double d = distances[std::make_pair(path.states[i], path.states[j])];
                if (d < minDist)
                {
                    minDist = d;
                    p1 = i;
                    p2 = j;
                }
            }

        if (p1 >= 0 && p2 >= 0)
        {
            if (si->checkMotion(path.states[p1], path.states[p2]))
            {
                for (int i = p1 + 1 ; i < p2 ; ++i)
                    si->freeState(path.states[i]);
                path.states.erase(path.states.begin() + p1 + 1, path.states.begin() + p2);
                nochange = 0;
            }
            else
                distances[std::make_pair(path.states[p1], path.states[p2])] = std::numeric_limits<double>::infinity();
        }
        else
            break;
    }
}

void ompl::geometric::PathSimplifier::simplifyMax(PathGeometric &path)
{
    reduceVertices(path);
    collapseCloseVertices(path);
    smoothBSpline(path, 5, path.length()/100.0);
    const std::pair<bool, bool> &p = path.checkAndRepair(magic::MAX_VALID_SAMPLE_ATTEMPTS);
    if (!p.second)
        msg_.warn("Solution path may slightly touch on an invalid region of the state space");
    else
        if (!p.first)
            msg_.debug("The solution path was slightly touching on an invalid region of the state space, but it was successfully fixed.");
}

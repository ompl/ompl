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
#include "ompl/util/MagicConstants.h"
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
                si->getStateManifold()->interpolate(path.states[i - 1], path.states[i], 0.5, temp1);
                si->getStateManifold()->interpolate(path.states[i], path.states[i + 1], 0.5, temp2);
                si->getStateManifold()->interpolate(temp1, temp2, 0.5, temp1);
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
            for (int i = p1 + 1 ; i < p2 ; ++i)
                si->freeState(path.states[i]);
            path.states.erase(path.states.begin() + p1 + 1, path.states.begin() + p2);
            nochange = 0;
        }
    }
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

    // compute pair-wise distances in path
    std::map<std::pair<const base::State*, const base::State*>, double> distances;
    for (unsigned int i = 0 ; i < path.states.size() ; ++i)
        for (unsigned int j = i + 1 ; j < path.states.size() ; ++j)
        {
            double d = si->distance(path.states[i], path.states[j]);
            distances[std::make_pair(path.states[i], path.states[j])] = d;
            distances[std::make_pair(path.states[j], path.states[i])] = d;
        }

    unsigned int nochange = 0;

    for (unsigned int s = 0 ; s < maxSteps && nochange < maxEmptySteps ; ++s, ++nochange)
    {
        // find closest pair of points
        double minDist = std::numeric_limits<double>::infinity();
        int p1 = -1;
        int p2 = -1;
        for (unsigned int i = 0 ; i < path.states.size() ; ++i)
            for (unsigned int j = i + 1 ; j < path.states.size() ; ++j)
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
    if (!path.checkAndRepair(magic::VALID_SAMPLE_ATTEMPTS))
	msg_.warn("Solution path may slightly touch on an invalid region of the state space");
}

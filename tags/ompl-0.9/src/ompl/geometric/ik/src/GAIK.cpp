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

#include "ompl/geometric/ik/GAIK.h"
#include "ompl/util/Time.h"
#include "ompl/util/Exception.h"
#include <algorithm>
#include <limits>

bool ompl::geometric::GAIK::solve(double solveTime, const base::GoalRegion &goal, base::State *result, const std::vector<base::State*> &hint)
{
    if (maxDistance_ < std::numeric_limits<double>::epsilon())
    {
        maxDistance_ = si_->getMaximumExtent() / 5.0;
        msg_.inform("The maximum distance for sampling around states is assumed to be %f", maxDistance_);
    }

    if (poolSize_ < 1)
    {
        msg_.error("Pool size too small");
        return false;
    }

    time::point             endTime = time::now() + time::seconds(solveTime);

    unsigned int            maxPoolSize = poolSize_ + poolExpansion_;
    std::vector<Individual> pool(maxPoolSize);
    IndividualSort          gs;
    bool                    solved = false;
    int                     solution = -1;

    base::ManifoldStateSamplerPtr sampler = si_->allocManifoldStateSampler();

    // add hint states
    unsigned int nh = std::min<unsigned int>(maxPoolSize, hint.size());
    for (unsigned int i = 0 ; i < nh ; ++i)
    {
        pool[i].state = si_->cloneState(hint[i]);
        si_->enforceBounds(pool[i].state);
        pool[i].valid = valid(pool[i].state);
        if (goal.isSatisfied(pool[i].state, &(pool[i].distance)))
        {
            if (pool[i].valid)
            {
                solved = true;
                solution = i;
            }
        }
    }

    // add states near the hint states
    unsigned int nh2 = nh * 2;
    if (nh2 < maxPoolSize)
    {
        for (unsigned int i = nh ; i < nh2 ; ++i)
        {
            pool[i].state = si_->allocState();
            sampler->sampleUniformNear(pool[i].state, pool[i % nh].state, maxDistance_);
            pool[i].valid = valid(pool[i].state);
            if (goal.isSatisfied(pool[i].state, &(pool[i].distance)))
            {
                if (pool[i].valid)
                {
                    solved = true;
                    solution = i;
                }
            }
        }
        nh = nh2;
    }

    // add random states
    for (unsigned int i = nh ; i < maxPoolSize ; ++i)
    {
        pool[i].state = si_->allocState();
        sampler->sampleUniform(pool[i].state);
        pool[i].valid = valid(pool[i].state);
        if (goal.isSatisfied(pool[i].state, &(pool[i].distance)))
        {
            if (pool[i].valid)
            {
                solved = true;
                solution = i;
            }
        }
    }

    // run the genetic algorithm
    unsigned int generations = 1;
    unsigned int mutationsSize = maxPoolSize - maxPoolSize % poolSize_;
    if (mutationsSize == 0)
        mutationsSize = maxPoolSize;
    if (mutationsSize == maxPoolSize)
        mutationsSize--;

    while (!solved && time::now() < endTime)
    {
        generations++;
        std::sort(pool.begin(), pool.end(), gs);

        // add mutations
        for (unsigned int i = poolSize_ ; i < mutationsSize ; ++i)
        {
            sampler->sampleUniformNear(pool[i].state, pool[i % poolSize_].state, maxDistance_);
            pool[i].valid = valid(pool[i].state);
            if (goal.isSatisfied(pool[i].state, &(pool[i].distance)))
            {
                if (pool[i].valid)
                {
                    solved = true;
                    solution = i;
                    break;
                }
            }
        }

        // add random states
        if (!solved)
            for (unsigned int i = mutationsSize ; i < maxPoolSize ; ++i)
            {
                sampler->sampleUniform(pool[i].state);
                pool[i].valid = valid(pool[i].state);
                if (goal.isSatisfied(pool[i].state, &(pool[i].distance)))
                {
                    if (pool[i].valid)
                    {
                        solved = true;
                        solution = i;
                        break;
                    }
                }
            }
    }


    // fill in solution, if found
    msg_.inform("Ran for %u generations", generations);

    if (solved)
    {
        // set the solution
        si_->copyState(result, pool[solution].state);

        // try to improve the solution
        tryToImprove(goal, result, pool[solution].distance);

        // if improving the state made it invalid, revert
        if (!valid(result))
            si_->copyState(result, pool[solution].state);
    }
    else
    {
        /* one last attempt to find a solution */
        std::sort(pool.begin(), pool.end(), gs);
        for (unsigned int i = 0 ; i < 5 ; ++i)
        {
            // get a valid state that is closer to the goal
            if (pool[i].valid)
            {
                // set the solution
                si_->copyState(result, pool[i].state);

                // try to improve the state
                tryToImprove(goal, result, pool[i].distance);

                // if the improvement made the state no longer valid, revert to previous one
                if (!valid(result))
                    si_->copyState(result, pool[i].state);
                else
                    solved = goal.isSatisfied(result);
                if (solved)
                    break;
            }
        }
    }

    for (unsigned int i = 0 ; i < maxPoolSize ; ++i)
        si_->freeState(pool[i].state);

    return solved;
}

bool ompl::geometric::GAIK::tryToImprove(const base::GoalRegion &goal, base::State *state, double distance)
{
    msg_.debug("Distance to goal before improvement: %g", distance);
    time::point start = time::now();
    double dist = si_->getMaximumExtent() / 10.0;
    hcik_.tryToImprove(goal, state, dist, &distance);
    hcik_.tryToImprove(goal, state, dist / 3.0, &distance);
    hcik_.tryToImprove(goal, state, dist / 10.0, &distance);
    msg_.debug("Improvement took  %u ms", (time::now() - start).total_milliseconds());
    msg_.debug("Distance to goal after improvement: %g", distance);
    return true;
}

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

#include "ompl/geometric/GeneticSearch.h"
#include "ompl/util/Time.h"
#include "ompl/util/Exception.h"
#include "ompl/tools/config/SelfConfig.h"
#include <algorithm>
#include <limits>

ompl::geometric::GeneticSearch::GeneticSearch(const base::SpaceInformationPtr &si)
  : hc_(si)
  , si_(si)
  , poolSize_(100)
  , poolMutation_(20)
  , poolRandom_(30)
  , generations_(0)
  , tryImprove_(false)
  , maxDistance_(0.0)
{
    hc_.setMaxImproveSteps(3);
    setValidityCheck(true);
}

ompl::geometric::GeneticSearch::~GeneticSearch()
{
    for (auto &i : pool_)
        si_->freeState(i.state);
}

bool ompl::geometric::GeneticSearch::solve(double solveTime, const base::GoalRegion &goal, base::State *result,
                                           const std::vector<base::State *> &hint)
{
    if (maxDistance_ < std::numeric_limits<double>::epsilon())
    {
        tools::SelfConfig sc(si_, "GeneticSearch");
        sc.configurePlannerRange(maxDistance_);
    }

    if (poolSize_ < 1)
    {
        OMPL_ERROR("Pool size too small");
        return false;
    }

    time::point endTime = time::now() + time::seconds(solveTime);

    unsigned int maxPoolSize = poolSize_ + poolMutation_ + poolRandom_;
    IndividualSort gs;
    bool solved = false;
    int solution = -1;

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    if (pool_.empty())
    {
        generations_ = 1;
        pool_.resize(maxPoolSize);
        // add hint states
        unsigned int nh = std::min<unsigned int>(maxPoolSize, hint.size());
        for (unsigned int i = 0; i < nh; ++i)
        {
            pool_[i].state = si_->cloneState(hint[i]);
            si_->enforceBounds(pool_[i].state);
            pool_[i].valid = valid(pool_[i].state);
            if (goal.isSatisfied(pool_[i].state, &(pool_[i].distance)))
            {
                if (pool_[i].valid)
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
            for (unsigned int i = nh; i < nh2; ++i)
            {
                pool_[i].state = si_->allocState();
                sampler_->sampleUniformNear(pool_[i].state, pool_[i % nh].state, maxDistance_);
                pool_[i].valid = valid(pool_[i].state);
                if (goal.isSatisfied(pool_[i].state, &(pool_[i].distance)))
                {
                    if (pool_[i].valid)
                    {
                        solved = true;
                        solution = i;
                    }
                }
            }
            nh = nh2;
        }

        // add random states
        for (unsigned int i = nh; i < maxPoolSize; ++i)
        {
            pool_[i].state = si_->allocState();
            sampler_->sampleUniform(pool_[i].state);
            pool_[i].valid = valid(pool_[i].state);
            if (goal.isSatisfied(pool_[i].state, &(pool_[i].distance)))
            {
                if (pool_[i].valid)
                {
                    solved = true;
                    solution = i;
                }
            }
        }
    }
    else
    {
        std::size_t initialSize = pool_.size();
        // free extra memory if needed
        for (std::size_t i = maxPoolSize; i < initialSize; ++i)
            si_->freeState(pool_[i].state);
        pool_.resize(maxPoolSize);
        // alloc extra memory if needed
        for (std::size_t i = initialSize; i < pool_.size(); ++i)
            pool_[i].state = si_->allocState();

        // add hint states at the bottom of the pool
        unsigned int nh = std::min<unsigned int>(maxPoolSize, hint.size());
        for (unsigned int i = 0; i < nh; ++i)
        {
            std::size_t pi = maxPoolSize - i - 1;
            si_->copyState(pool_[pi].state, hint[i]);
            si_->enforceBounds(pool_[pi].state);
            pool_[pi].valid = valid(pool_[pi].state);
            if (goal.isSatisfied(pool_[pi].state, &(pool_[pi].distance)))
            {
                if (pool_[pi].valid)
                {
                    solved = true;
                    solution = pi;
                }
            }
        }

        // add random states if needed
        nh = maxPoolSize - nh;
        for (std::size_t i = initialSize; i < nh; ++i)
        {
            sampler_->sampleUniform(pool_[i].state);
            pool_[i].valid = valid(pool_[i].state);
            if (goal.isSatisfied(pool_[i].state, &(pool_[i].distance)))
            {
                if (pool_[i].valid)
                {
                    solved = true;
                    solution = i;
                }
            }
        }
    }

    // run the genetic algorithm
    unsigned int mutationsSize = poolSize_ + poolMutation_;

    while (!solved && time::now() < endTime)
    {
        generations_++;
        std::sort(pool_.begin(), pool_.end(), gs);

        // add mutations
        for (unsigned int i = poolSize_; i < mutationsSize; ++i)
        {
            sampler_->sampleUniformNear(pool_[i].state, pool_[i % poolSize_].state, maxDistance_);
            pool_[i].valid = valid(pool_[i].state);
            if (goal.isSatisfied(pool_[i].state, &(pool_[i].distance)))
            {
                if (pool_[i].valid)
                {
                    solved = true;
                    solution = i;
                    break;
                }
            }
        }

        // add random states
        if (!solved)
            for (unsigned int i = mutationsSize; i < maxPoolSize; ++i)
            {
                sampler_->sampleUniform(pool_[i].state);
                pool_[i].valid = valid(pool_[i].state);
                if (goal.isSatisfied(pool_[i].state, &(pool_[i].distance)))
                {
                    if (pool_[i].valid)
                    {
                        solved = true;
                        solution = i;
                        break;
                    }
                }
            }
    }

    // fill in solution, if found
    OMPL_INFORM("Ran for %u generations", generations_);

    if (solved)
    {
        // set the solution
        si_->copyState(result, pool_[solution].state);

        // try to improve the solution
        if (tryImprove_)
            tryToImprove(goal, result, pool_[solution].distance);

        // if improving the state made it invalid, revert
        if (!valid(result))
            si_->copyState(result, pool_[solution].state);
    }
    else if (tryImprove_)
    {
        /* one last attempt to find a solution */
        std::sort(pool_.begin(), pool_.end(), gs);
        for (unsigned int i = 0; i < 5; ++i)
        {
            // get a valid state that is closer to the goal
            if (pool_[i].valid)
            {
                // set the solution
                si_->copyState(result, pool_[i].state);

                // try to improve the state
                tryToImprove(goal, result, pool_[i].distance);

                // if the improvement made the state no longer valid, revert to previous one
                if (!valid(result))
                    si_->copyState(result, pool_[i].state);
                else
                    solved = goal.isSatisfied(result);
                if (solved)
                    break;
            }
        }
    }

    return solved;
}

void ompl::geometric::GeneticSearch::tryToImprove(const base::GoalRegion &goal, base::State *state, double distance)
{
    OMPL_DEBUG("Distance to goal before improvement: %g", distance);
    time::point start = time::now();
    double dist = si_->getMaximumExtent() / 10.0;
    hc_.tryToImprove(goal, state, dist, &distance);
    hc_.tryToImprove(goal, state, dist / 3.0, &distance);
    hc_.tryToImprove(goal, state, dist / 10.0, &distance);
    OMPL_DEBUG("Improvement took  %u ms",
               std::chrono::duration_cast<std::chrono::milliseconds>(time::now() - start).count());
    OMPL_DEBUG("Distance to goal after improvement: %g", distance);
}

void ompl::geometric::GeneticSearch::clear()
{
    generations_ = 0;
    pool_.clear();
    sampler_.reset();
}

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

#include "ompl/geometric/planners/rrt/pRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <boost/thread/thread.hpp>
#include <limits>

ompl::geometric::pRRT::pRRT(const base::SpaceInformationPtr &si) : base::Planner(si, "pRRT"),
                                                                  samplerArray_(si)
{
    specs_.approximateSolutions = true;
    specs_.multithreaded = true;
    specs_.directed = true;

    setThreadCount(2);
    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    lastGoalMotion_ = NULL;

    Planner::declareParam<double>("range", this, &pRRT::setRange, &pRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &pRRT::setGoalBias, &pRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<unsigned int>("thread_count", this, &pRRT::setThreadCount, &pRRT::getThreadCount, "1:64");
}

ompl::geometric::pRRT::~pRRT()
{
    freeMemory();
}

void ompl::geometric::pRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&pRRT::distanceFunction, this, _1, _2));
}

void ompl::geometric::pRRT::clear()
{
    Planner::clear();
    samplerArray_.clear();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = NULL;
}

void ompl::geometric::pRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

void ompl::geometric::pRRT::threadSolve(unsigned int tid, const base::PlannerTerminationCondition &ptc, SolutionInfo *sol)
{
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
    RNG                         rng;

    Motion *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (sol->solution == NULL && ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            samplerArray_[tid]->sampleUniform(rstate);

        /* find closest state in the tree */
        nnLock_.lock();
        Motion *nmotion = nn_->nearest(rmotion);
        nnLock_.unlock();
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        if (si_->checkMotion(nmotion->state, dstate))
        {
            /* create a motion */
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;

            nnLock_.lock();
            nn_->add(motion);
            nnLock_.unlock();

            double dist = 0.0;
            bool solved = goal->isSatisfied(motion->state, &dist);
            if (solved)
            {
                sol->lock.lock();
                sol->approxdif = dist;
                sol->solution = motion;
                sol->lock.unlock();
                break;
            }
            if (dist < sol->approxdif)
            {
                sol->lock.lock();
                if (dist < sol->approxdif)
                {
                    sol->approxdif = dist;
                    sol->approxsol = motion;
                }
                sol->lock.unlock();
            }
        }
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;
}

ompl::base::PlannerStatus ompl::geometric::pRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    base::GoalRegion *goal = dynamic_cast<base::GoalRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        OMPL_ERROR("%s: Unknow type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    samplerArray_.resize(threadCount_);

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    SolutionInfo sol;
    sol.solution = NULL;
    sol.approxsol = NULL;
    sol.approxdif = std::numeric_limits<double>::infinity();

    std::vector<boost::thread*> th(threadCount_);
    for (unsigned int i = 0 ; i < threadCount_ ; ++i)
        th[i] = new boost::thread(boost::bind(&pRRT::threadSolve, this, i, ptc, &sol));
    for (unsigned int i = 0 ; i < threadCount_ ; ++i)
    {
        th[i]->join();
        delete th[i];
    }

    bool solved = false;
    bool approximate = false;
    if (sol.solution == NULL)
    {
        sol.solution = sol.approxsol;
        approximate = true;
    }

    if (sol.solution != NULL)
    {
        lastGoalMotion_ = sol.solution;

        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (sol.solution != NULL)
        {
            mpath.push_back(sol.solution);
            sol.solution = sol.solution->parent;
        }

        /* set the solution path */
        PathGeometric *path = new PathGeometric(si_);
           for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->append(mpath[i]->state);

        pdef_->addSolutionPath(base::PathPtr(path), approximate, sol.approxdif, getName());
        solved = true;
    }

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::pRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                         base::PlannerDataVertex(motions[i]->state));
    }
}

void ompl::geometric::pRRT::setThreadCount(unsigned int nthreads)
{
    assert(nthreads > 0);
    threadCount_ = nthreads;
}

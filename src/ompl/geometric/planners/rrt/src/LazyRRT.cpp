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

#include "ompl/geometric/planners/rrt/LazyRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <cassert>

ompl::geometric::LazyRRT::LazyRRT(const base::SpaceInformationPtr &si) : base::Planner(si, "LazyRRT")
{
    specs_.directed = true;
    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    lastGoalMotion_ = NULL;

    Planner::declareParam<double>("range", this, &LazyRRT::setRange, &LazyRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &LazyRRT::setGoalBias, &LazyRRT::getGoalBias, "0.:.05:1.");
}

ompl::geometric::LazyRRT::~LazyRRT()
{
    freeMemory();
}

void ompl::geometric::LazyRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&LazyRRT::distanceFunction, this, _1, _2));
}

void ompl::geometric::LazyRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = NULL;
}

void ompl::geometric::LazyRRT::freeMemory()
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

ompl::base::PlannerStatus ompl::geometric::LazyRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->valid = true;
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = NULL;
    double  distsol  = -1.0;
    Motion *rmotion  = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    bool solutionFound = false;

    while (ptc == false && !solutionFound)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        assert(nmotion != rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        /* create a motion */
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
        motion->parent = nmotion;
        nmotion->children.push_back(motion);
        nn_->add(motion);

        double dist = 0.0;
        if (goal->isSatisfied(motion->state, &dist))
        {
            distsol = dist;
            solution = motion;
            solutionFound = true;
            lastGoalMotion_ = solution;

            // Check that the solution is valid:
            // construct the solution path
            std::vector<Motion*> mpath;
            while (solution != NULL)
            {
                mpath.push_back(solution);
                solution = solution->parent;
            }

            // check each segment along the path for validity
            for (int i = mpath.size() - 1 ; i >= 0 && solutionFound; --i)
                if (!mpath[i]->valid)
                {
                    if (si_->checkMotion(mpath[i]->parent->state, mpath[i]->state))
                        mpath[i]->valid = true;
                    else
                    {
                        removeMotion(mpath[i]);
                        solutionFound = false;
                        lastGoalMotion_ = NULL;
                    }
                }

            if (solutionFound)
            {
                // set the solution path
                PathGeometric *path = new PathGeometric(si_);
                for (int i = mpath.size() - 1 ; i >= 0 ; --i)
                    path->append(mpath[i]->state);

                pdef_->addSolutionPath(base::PathPtr(path), false, distsol, getName());
            }
        }
    }

    si_->freeState(xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return solutionFound ?  base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::LazyRRT::removeMotion(Motion *motion)
{
    nn_->remove(motion);

    /* remove self from parent list */

    if (motion->parent)
    {
        for (unsigned int i = 0 ; i < motion->parent->children.size() ; ++i)
            if (motion->parent->children[i] == motion)
            {
                motion->parent->children.erase(motion->parent->children.begin() + i);
                break;
            }
    }

    /* remove children */
    for (unsigned int i = 0 ; i < motion->children.size() ; ++i)
    {
        motion->children[i]->parent = NULL;
        removeMotion(motion->children[i]);
    }

    if (motion->state)
        si_->freeState(motion->state);
    delete motion;
}

void ompl::geometric::LazyRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state, 1));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parent ? motions[i]->parent->state : NULL),
                         base::PlannerDataVertex(motions[i]->state));

        data.tagState(motions[i]->state, motions[i]->valid ? 1 : 0);
    }
}

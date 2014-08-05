/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Ryan Luna */

#include "ompl/geometric/planners/rrt/ConstrainedRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

ompl::geometric::ConstrainedRRT::ConstrainedRRT(const base::SpaceInformationPtr &si) : base::Planner(si, "ConstrainedRRT")
{
    specs_.approximateSolutions = false;
    specs_.directed = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    lastGoalMotion_ = NULL;

    const base::ConstrainedSpaceInformationPtr& csi = boost::dynamic_pointer_cast<base::ConstrainedSpaceInformation>(si);
    if (!csi)
        OMPL_ERROR("%s: Failed to cast SpaceInformation to ConstrainedSpaceInformation", getName().c_str());
    else
        ci_ = csi->getConstraintInformation();

    Planner::declareParam<double>("range", this, &ConstrainedRRT::setRange, &ConstrainedRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &ConstrainedRRT::setGoalBias, &ConstrainedRRT::getGoalBias, "0.:.05:1.");
}

ompl::geometric::ConstrainedRRT::~ConstrainedRRT(void)
{
    freeMemory();
}

void ompl::geometric::ConstrainedRRT::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = NULL;
}

void ompl::geometric::ConstrainedRRT::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&ConstrainedRRT::distanceFunction, this, _1, _2));
}

void ompl::geometric::ConstrainedRRT::freeMemory(void)
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

bool ompl::geometric::ConstrainedRRT::constrainedExtend(const base::State* a, const base::State* b,
                                                        std::vector<base::State*>& result) const
{
    // Assuming a is valid and on constraint manifold
    base::StateSpacePtr ss = si_->getStateSpace();

    int n = 20*ss->validSegmentCount(a, b);  // number of steps between a and b in the state space
    double step = 1;

    double prevDist = std::numeric_limits<double>::max();
    while (true)
    {
        base::State* scratchState = ss->allocState();
        ss->interpolate(a, b, step++/n, scratchState);

        // Project new state onto constraint manifold
        if (!ci_->project(scratchState) || !si_->isValid(scratchState))
        {
            ss->freeState(scratchState);
            break;
        }

        // Check for divergence
        double d = ss->distance(scratchState, b);
        if (d > prevDist)
        {
            ss->freeState(scratchState);
            break;
        }
        prevDist = d;

        // No divergence; getting closer.  Store the new state
        result.push_back(scratchState);

        // Done.  This will probably never happen
        if (ss->equalStates(scratchState, b))
            break;
    }

    return result.size() > 0;
}

ompl::base::PlannerStatus ompl::geometric::ConstrainedRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    if (!ci_)
    {
        OMPL_ERROR("%s: ConstraintInformation is invalid!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

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

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    if (ci_->numConstraints() == 0)
    {
        OMPL_WARN("%s: There are no constraints defined!", getName().c_str());
    }

    OMPL_INFORM("%s: Starting with %u states", getName().c_str(), nn_->size());

    Motion *solution  = NULL;
    Motion *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    base::StateSpacePtr ss = si_->getStateSpace();
    std::vector<base::State*> extension;
    bool sat = false;

    while (ptc == false && !sat)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            ss->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        extension.clear();
        if (constrainedExtend(nmotion->state, dstate, extension))
        {
            Motion* parent = nmotion;

            // Create a new motion for the sequence of states in the extension
            for(size_t i = 0; i < extension.size(); ++i)
            {
                Motion *motion = new Motion(extension[i], parent);
                parent = motion;

                nn_->add(motion);

                sat = goal->isSatisfied(motion->state);
                if (sat)
                {
                    solution = motion;

                    // Free remaining states in list, then break
                    for (size_t j = i+1; j < extension.size(); ++j)
                        ss->freeState(extension[j]);
                    break;
                }
            }
        }

        /*if (si_->checkMotion(nmotion->state, dstate))
        {
            // create a motion
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;

            nn_->add(motion);
            double dist = 0.0;
            bool sat = goal->isSatisfied(motion->state, &dist);  // remove this distance call
            if (sat)
            {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        }*/
    }

    if (sat)
    {
        lastGoalMotion_ = solution;

        // Construct the solution path by working backward from solution
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // Store the solution path
        PathGeometric *path = new PathGeometric(si_);
           for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(base::PathPtr(path), false, 0.0);
    }




    /*bool solved = false;
    bool approximate = false;
    if (solution == NULL)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != NULL)
    {
        lastGoalMotion_ = solution;

        // construct the solution path
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // set the solution path
        PathGeometric *path = new PathGeometric(si_);
           for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif);
        solved = true;
    }*/

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(sat, false);
}

void ompl::geometric::ConstrainedRRT::getPlannerData(base::PlannerData &data) const
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

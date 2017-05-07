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

/* Author: Alexander Jiang */

#include "ompl/geometric/planners/rrt/RRTplus.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/Exception.h"
<<<<<<< HEAD
#include <chrono>
=======
#include "ompl/util/Time.h"
>>>>>>> 864c6839b03729ce569519ae8de92573a6953522
#include <cmath>
#include <limits>

ompl::geometric::RRTPlus::RRTPlus(const base::SpaceInformationPtr &si) : base::Planner(si, "RRT+")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    lastGoalMotion_ = nullptr;

    Planner::declareParam<double>("range", this, &RRTPlus::setRange, &RRTPlus::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTPlus::setGoalBias, &RRTPlus::getGoalBias, "0.:.05:1.");
    // The paper describes that this parameter was specified by indicating the total desired time for the subsearch
    // instead of the max number of samples...
    Planner::declareParam<double>("subsearch_bound", this, &RRTPlus::setSubsearchBound, &RRTPlus::getSubsearchBound, "");
}

ompl::geometric::RRTPlus::~RRTPlus()
{
    freeMemory();
}

void ompl::geometric::RRTPlus::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::RRTPlus::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    // Try to dynamic_cast to CompoundStateSpace.
    // auto *test = dynamic_cast<ompl::base::CompoundStateSpace>(si_->getStateSpace());
    // if (test == NULL)
    //     throw new Exception("Failed to cast state space to a ompl::base::CompoundStateSpace.");
    if (!si_->getStateSpace()->isCompound())
        throw new Exception("State space is not a ompl::base::CompoundStateSpace.");

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });
}

void ompl::geometric::RRTPlus::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

// Change this method.
ompl::base::PlannerStatus ompl::geometric::RRTPlus::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    // don't need goal biasing for v1.0
//    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // Note that the state space in si_->getStateSpace() must be a
    // CompoundStateSpace due to the check in setup().

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

<<<<<<< HEAD
    si_->getStateSpace()->setStateSamplerAllocator(ConstrainedSubspaceStateSampler::allocConstrainedSubspaceStateSampler);
    sampler_ = si_->getStateSpace()->allocStateSampler();
=======
    // si_->getStateSpace()->setStateSamplerAllocator(ConstrainedSubspaceStateSampler::allocConstrainedSubspaceStateSampler);
    // sampler_ = si_->getStateSpace()->allocStateSampler();
    sampler_ = ConstrainedSubspaceStateSampler::allocConstrainedSubspaceStateSampler(si_->getStateSpace().get());
>>>>>>> 864c6839b03729ce569519ae8de92573a6953522
    sampler_->constrainAllComponents();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity(); // what is this for?
    auto *rmotion = new Motion(si_); // how to get the inital state to be along the line btwn q_init and q_goal?
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
    unsigned int n = 1;
    base::PlannerData plannerData(si_);
    getPlannerData(plannerData);
    const base::State *q_init = plannerData.getStartVertex(0).getState();
    const base::State *q_goal = plannerData.getGoalVertex(0).getState();

    while (ptc == false && n <= si_->getStateSpace()->getDimension())
    {
        double subsearch_duration = pow(exp(log(getSubsearchBound()) / n), n);
        time::point end = time::now() + time::seconds(subsearch_duration);
        while (time::now() < end) {
            // note for improvements: look into the planner termination condition for limiting # of samples for each subsearch

            // don't need goal biasing for v1.0
            // /* sample random state (with goal biasing) */
            // if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            //     goal_s->sampleGoal(rstate);
            // else
            //     sampler_->sampleUniform(rstate);

            // For prioritized sampling, we need a state on the line between q_init and q_goal
            si_->getStateSpace()->interpolate(q_init, q_goal, ((double) rand() / (RAND_MAX)), rstate);

            // For prioritized sampling, only need to cast the sampled state to a CompoundState
            // auto *xstate = xstate-><ompl::base:CompoundStateSpace::StateType>as();

            sampler_->sampleUniform(rstate);

            /* find closest state in the tree */
            Motion *nmotion = nn_->nearest(rmotion);
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
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, dstate);
                motion->parent = nmotion;

                nn_->add(motion);
                double dist = 0.0;
                bool sat = goal->isSatisfied(motion->state, &dist);
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
            }
            sampler_->unconstrainComponent(n-1);
            n++;
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::RRTPlus::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

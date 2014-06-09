/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Oren Salzman
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

/* Author: Oren Salzman, Sertac Karaman, Ioan Sucan */

#include "ompl/geometric/planners/rrt/LBTRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <math.h>

const double ompl::geometric::LBTRRT::kRRG = 5.5;

ompl::geometric::LBTRRT::LBTRRT(const base::SpaceInformationPtr &si)
    : base::Planner(si, "LBTRRT")
{

    specs_.approximateSolutions = true;
    specs_.directed = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    lastGoalMotion_ = NULL;
    epsilon_ = 0.4;


    Planner::declareParam<double>("range", this, &LBTRRT::setRange, &LBTRRT::getRange);
    Planner::declareParam<double>("goal_bias", this, &LBTRRT::setGoalBias, &LBTRRT::getGoalBias);
    Planner::declareParam<double>("epsilon", this, &LBTRRT::setApproximationFactor, &LBTRRT::getApproximationFactor);
}

ompl::geometric::LBTRRT::~LBTRRT()
{
    freeMemory();
}

void ompl::geometric::LBTRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = NULL;
}

void ompl::geometric::LBTRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(new NearestNeighborsGNAT<Motion*>());
    nn_->setDistanceFunction(boost::bind(&LBTRRT::distanceFunction, this, _1, _2));
}

void ompl::geometric::LBTRRT::freeMemory()
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

ompl::base::PlannerStatus ompl::geometric::LBTRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
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
        OMPL_ERROR("There are no valid initial states!");
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("Starting with %u states", nn_->size());

    Motion *solution  = NULL;
    Motion *approxSol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();

    Motion *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while ( ptc() == false )
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
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        if (si_->checkMotion(nmotion->state, dstate))
        {
            /* create a motion */
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);

            /* update fields */
            motion->parentLb_ = nmotion;
            motion->parentApx_ = nmotion;
            double distN = distanceFunction(nmotion, motion);
            motion->costLb_ = nmotion->costLb_ + distN;
            motion->costApx_ = nmotion->costApx_ + distN;

            nmotion->childrenLb_.push_back(motion);
            nmotion->childrenApx_.push_back(motion);

            nn_->add(motion);

            /* do lazy rewiring */
            double k = std::log(double(nn_->size())) * kRRG;
            std::vector<Motion *> nnVec;
            nn_->nearestK(rmotion, static_cast<int>(k), nnVec);

            IsLessThan  isLessThan(this,motion);
            std::sort (nnVec.begin(), nnVec.end(), isLessThan);

            for (std::size_t i = 0; i < nnVec.size(); ++i)
                attemptNodeUpdate(motion, nnVec[i]);

            for (std::size_t i = 0; i < nnVec.size(); ++i)
                attemptNodeUpdate(nnVec[i], motion);

            double dist = 0.0;
            bool sat = goal->isSatisfied(motion->state, &dist);
            if (sat || dist < approxdif)
            {
                approxdif = dist;
                solution = motion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;

    if (solution == NULL)
    {
        solution = approxSol;
        approximate = true;
    }

    if (solution != NULL)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parentApx_;
        }

        /* set the solution path */
        PathGeometric *path = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif);
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("Created %u states", nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::LBTRRT::attemptNodeUpdate(Motion *potentialParent, Motion *child)
{
    double dist = distanceFunction(potentialParent, child);
    double potentialLb = potentialParent->costLb_ + dist;
    double potentialApx = potentialParent->costApx_ + dist;

    if (child->costLb_ <= potentialLb)
        return;

    if (child->costApx_ > 1.0 + epsilon_ *  potentialLb)
    {
        if (si_->checkMotion(potentialParent->state, child->state) == false)
            return;

        removeFromParentLb(child);
        double deltaLb = potentialLb - child->costLb_;
        child->parentLb_ = potentialParent;
        potentialParent->childrenLb_.push_back(child);
        child->costLb_ = potentialLb;
        updateChildCostsLb(child, deltaLb);


        if (child->costApx_ <= potentialApx)
            return;

        removeFromParentApx(child);
        double deltaApx = potentialApx - child->costApx_;
        child->parentApx_ = potentialParent;
        potentialParent->childrenApx_.push_back(child);
        child->costApx_ = potentialApx;
        updateChildCostsApx(child, deltaApx);
    }
    else //(child->costApx_ <= 1 + epsilon_ *  potentialLb)
    {
        removeFromParentLb(child);
        double deltaLb = potentialLb - child->costLb_;
        child->parentLb_ = potentialParent;
        potentialParent->childrenLb_.push_back(child);
        child->costLb_ = potentialLb;
        updateChildCostsLb(child, deltaLb);
    }

    return;
}

void ompl::geometric::LBTRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parentApx_ == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parentApx_->state),
                         base::PlannerDataVertex(motions[i]->state));
    }
}


void ompl::geometric::LBTRRT::updateChildCostsLb(Motion *m, double delta)
{
    for (size_t i = 0; i < m->childrenLb_.size(); ++i)
    {
        m->childrenLb_[i]->costLb_ += delta;
        updateChildCostsLb(m->childrenLb_[i], delta);
    }
}
void ompl::geometric::LBTRRT::updateChildCostsApx(Motion *m, double delta)
{
    for (size_t i = 0; i < m->childrenApx_.size(); ++i)
    {
        m->childrenApx_[i]->costApx_ += delta;
        updateChildCostsApx(m->childrenApx_[i], delta);
    }
}

void ompl::geometric::LBTRRT::removeFromParentLb(Motion *m)
{
    return removeFromParent(m, m->parentLb_->childrenLb_);
}
void ompl::geometric::LBTRRT::removeFromParentApx(Motion *m)
{
    return removeFromParent(m, m->parentApx_->childrenApx_);
}
void ompl::geometric::LBTRRT::removeFromParent(const Motion *m, std::vector<Motion*>& vec)
{
    std::vector<Motion*>::iterator it = vec.begin ();
    while (it != vec.end ())
    {
        if (*it == m)
        {
            it = vec.erase(it);
            it = vec.end ();
        }
        else
            ++it;
    }
}

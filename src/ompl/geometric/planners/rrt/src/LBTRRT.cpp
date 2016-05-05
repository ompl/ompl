/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Tel Aviv University
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
*   * Neither the name of the Tel Aviv University nor the names of its
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

/* Author: Oren Salzman, Sertac Karaman, Ioan Sucan, Mark Moll */

#include "ompl/geometric/planners/rrt/LBTRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <math.h>
#include <boost/math/constants/constants.hpp>

ompl::geometric::LBTRRT::LBTRRT(const base::SpaceInformationPtr &si)
  : base::Planner(si, "LBTRRT")
  , goalBias_(0.05)
  , maxDistance_(0.0)
  , epsilon_(0.4)
  , lastGoalMotion_(nullptr)
  , iterations_(0)
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &LBTRRT::setRange, &LBTRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &LBTRRT::setGoalBias, &LBTRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("epsilon", this, &LBTRRT::setApproximationFactor, &LBTRRT::getApproximationFactor,
                                  "0.:.1:10.");

    addPlannerProgressProperty("iterations INTEGER", std::bind(&LBTRRT::getIterationCount, this));
    addPlannerProgressProperty("best cost REAL", std::bind(&LBTRRT::getBestCost, this));
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
    lastGoalMotion_ = nullptr;

    iterations_ = 0;
    bestCost_ = std::numeric_limits<double>::infinity();
}

void ompl::geometric::LBTRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction(std::bind(&LBTRRT::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));
}

void ompl::geometric::LBTRRT::freeMemory()
{
    if (idToMotionMap_.size() > 0)
    {
        for (unsigned int i = 0; i < idToMotionMap_.size(); ++i)
        {
            if (idToMotionMap_[i]->state_)
                si_->freeState(idToMotionMap_[i]->state_);
            delete idToMotionMap_[i];
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::LBTRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    // update goal and check validity
    base::Goal *goal = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    if (!goal)
    {
        OMPL_ERROR("%s: Goal undefined", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // update start and check validity
    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state_, st);
        motion->id_ = nn_->size();
        idToMotionMap_.push_back(motion);
        nn_->add(motion);
        lowerBoundGraph_.addVertex(motion->id_);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (nn_->size() > 1)
    {
        OMPL_ERROR("%s: There are multiple start states - currently not supported!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = lastGoalMotion_;
    Motion *approxSol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    // e*(1+1/d)  K-nearest constant, as used in RRT*
    double k_rrg =
        boost::math::constants::e<double>() + boost::math::constants::e<double>() / (double)si_->getStateDimension();

    Motion *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state_;
    base::State *xstate = si_->allocState();
    unsigned int statesGenerated = 0;

    bestCost_ = lastGoalMotion_ ? lastGoalMotion_->costApx_ : std::numeric_limits<double>::infinity();
    while (ptc() == false)
    {
        iterations_++;
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state_, rstate);
        if (d == 0)  // this takes care of the case that the goal is a single point and we re-sample it multiple times
            continue;
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state_, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        if (checkMotion(nmotion->state_, dstate))
        {
            statesGenerated++;
            /* create a motion */
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state_, dstate);

            /* update fields */
            double distN = distanceFunction(nmotion, motion);

            motion->id_ = nn_->size();
            idToMotionMap_.push_back(motion);
            lowerBoundGraph_.addVertex(motion->id_);
            motion->parentApx_ = nmotion;

            std::list<std::size_t> dummy;
            lowerBoundGraph_.addEdge(nmotion->id_, motion->id_, distN, false, dummy);

            motion->costLb_ = nmotion->costLb_ + distN;
            motion->costApx_ = nmotion->costApx_ + distN;
            nmotion->childrenApx_.push_back(motion);

            std::vector<Motion *> nnVec;
            unsigned int k = std::ceil(k_rrg * log((double)(nn_->size() + 1)));
            nn_->nearestK(motion, k, nnVec);
            nn_->add(motion);  // if we add the motion before the nearestK call, we will get ourselves...

            IsLessThan isLessThan(this, motion);
            std::sort(nnVec.begin(), nnVec.end(), isLessThan);

            //-------------------------------------------------//
            //  Rewiring Part (i) - find best parent of motion //
            //-------------------------------------------------//
            if (motion->parentApx_ != nnVec.front())
            {
                for (std::size_t i(0); i < nnVec.size(); ++i)
                {
                    Motion *potentialParent = nnVec[i];
                    double dist = distanceFunction(potentialParent, motion);
                    considerEdge(potentialParent, motion, dist);
                }
            }

            //------------------------------------------------------------------//
            //  Rewiring Part (ii)                                              //
            //  check if motion may be a better parent to one of its neighbors  //
            //------------------------------------------------------------------//
            for (std::size_t i(0); i < nnVec.size(); ++i)
            {
                Motion *child = nnVec[i];
                double dist = distanceFunction(motion, child);
                considerEdge(motion, child, dist);
            }

            double dist = 0.0;
            bool sat = goal->isSatisfied(motion->state_, &dist);

            if (sat)
            {
                approxdif = dist;
                solution = motion;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxSol = motion;
            }

            if (solution != nullptr && bestCost_ != solution->costApx_)
            {
                OMPL_INFORM("%s: approximation cost = %g", getName().c_str(), solution->costApx_);
                bestCost_ = solution->costApx_;
            }
        }
    }

    bool solved = false;
    bool approximate = false;

    if (solution == nullptr)
    {
        solution = approxSol;
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
            solution = solution->parentApx_;
        }

        /* set the solution path */
        PathGeometric *path = new PathGeometric(si_);
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state_);
        // Add the solution path.
        base::PathPtr bpath(path);
        base::PlannerSolution psol(bpath);
        psol.setPlannerName(getName());
        if (approximate)
            psol.setApproximate(approxdif);
        pdef_->addSolutionPath(psol);
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state_)
        si_->freeState(rmotion->state_);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), statesGenerated);

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::LBTRRT::considerEdge(Motion *parent, Motion *child, double c)
{
    // optimization - check if the bounded approximation invariant
    // will be violated after the edge insertion (at least for the child node)
    // if this is the case - perform the local planning
    // this prevents the update of the graph due to the edge insertion and then the re-update as it is removed
    double potential_cost = parent->costLb_ + c;
    if (child->costApx_ > (1 + epsilon_) * potential_cost)
        if (!checkMotion(parent, child))
            return;

    // update lowerBoundGraph_
    std::list<std::size_t> affected;

    lowerBoundGraph_.addEdge(parent->id_, child->id_, c, true, affected);

    // now, check if the bounded apprimation invariant has been violated for each affected vertex
    // insert them into a priority queue ordered according to the lb cost
    std::list<std::size_t>::iterator iter;
    IsLessThanLB isLessThanLB(this);
    Lb_queue queue(isLessThanLB);

    for (iter = affected.begin(); iter != affected.end(); ++iter)
    {
        Motion *m = getMotion(*iter);
        m->costLb_ = lowerBoundGraph_.getShortestPathCost(*iter);
        if (m->costApx_ > (1 + epsilon_) * m->costLb_)
            queue.insert(m);
    }

    while (queue.empty() == false)
    {
        Motion *motion = *(queue.begin());
        queue.erase(queue.begin());

        if (motion->costApx_ > (1 + epsilon_) * motion->costLb_)
        {
            Motion *potential_parent = getMotion(lowerBoundGraph_.getShortestPathParent(motion->id_));
            if (checkMotion(potential_parent, motion))
            {
                double delta = lazilyUpdateApxParent(motion, potential_parent);
                updateChildCostsApx(motion, delta);
            }
            else
            {
                affected.clear();

                lowerBoundGraph_.removeEdge(potential_parent->id_, motion->id_, true, affected);

                for (iter = affected.begin(); iter != affected.end(); ++iter)
                {
                    Motion *affected = getMotion(*iter);
                    Lb_queue_iter lb_queue_iter = queue.find(affected);
                    if (lb_queue_iter != queue.end())
                    {
                        queue.erase(lb_queue_iter);
                        affected->costLb_ = lowerBoundGraph_.getShortestPathCost(affected->id_);
                        if (affected->costApx_ > (1 + epsilon_) * affected->costLb_)
                            queue.insert(affected);
                    }
                    else
                    {
                        affected->costLb_ = lowerBoundGraph_.getShortestPathCost(affected->id_);
                    }
                }

                motion->costLb_ = lowerBoundGraph_.getShortestPathCost(motion->id_);
                if (motion->costApx_ > (1 + epsilon_) * motion->costLb_)
                    queue.insert(motion);

                // optimization - we can remove the opposite edge
                lowerBoundGraph_.removeEdge(motion->id_, potential_parent->id_, false, affected);
            }
        }
    }

    return;
}

void ompl::geometric::LBTRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state_));

    for (unsigned int i = 0; i < motions.size(); ++i)
    {
        if (motions[i]->parentApx_ == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state_));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parentApx_->state_),
                         base::PlannerDataVertex(motions[i]->state_));
    }
}

void ompl::geometric::LBTRRT::updateChildCostsApx(Motion *m, double delta)
{
    for (std::size_t i = 0; i < m->childrenApx_.size(); ++i)
    {
        Motion *child = m->childrenApx_[i];
        child->costApx_ += delta;
        updateChildCostsApx(child, delta);
    }
}

double ompl::geometric::LBTRRT::lazilyUpdateApxParent(Motion *child, Motion *parent)
{
    double dist = distanceFunction(parent, child);
    removeFromParentApx(child);
    double deltaApx = parent->costApx_ + dist - child->costApx_;
    child->parentApx_ = parent;
    parent->childrenApx_.push_back(child);
    child->costApx_ = parent->costApx_ + dist;

    return deltaApx;
}

void ompl::geometric::LBTRRT::removeFromParentApx(Motion *m)
{
    std::vector<Motion *> &vec = m->parentApx_->childrenApx_;
    for (std::vector<Motion *>::iterator it = vec.begin(); it != vec.end(); ++it)
        if (*it == m)
        {
            vec.erase(it);
            break;
        }
}

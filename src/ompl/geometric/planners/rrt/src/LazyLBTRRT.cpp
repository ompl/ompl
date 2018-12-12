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

/* Author: Oren Salzman, Mark Moll */

#include "ompl/geometric/planners/rrt/LazyLBTRRT.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

namespace
{
    int getK(unsigned int n, double k_rrg)
    {
        return std::ceil(k_rrg * log((double)(n + 1)));
    }
}

ompl::geometric::LazyLBTRRT::LazyLBTRRT(const base::SpaceInformationPtr &si)
  : base::Planner(si, "LazyLBTRRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &LazyLBTRRT::setRange, &LazyLBTRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &LazyLBTRRT::setGoalBias, &LazyLBTRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("epsilon", this, &LazyLBTRRT::setApproximationFactor,
                                  &LazyLBTRRT::getApproximationFactor, "0.:.1:10.");

    addPlannerProgressProperty("iterations INTEGER", [this]
                               {
                                   return getIterationCount();
                               });
    addPlannerProgressProperty("best cost REAL", [this]
                               {
                                   return getBestCost();
                               });
}

ompl::geometric::LazyLBTRRT::~LazyLBTRRT()
{
    freeMemory();
}

void ompl::geometric::LazyLBTRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    graphLb_.clear();
    graphApx_.clear();
    lastGoalMotion_ = nullptr;

    iterations_ = 0;
    bestCost_ = std::numeric_limits<double>::infinity();
}

void ompl::geometric::LazyLBTRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });
}

void ompl::geometric::LazyLBTRRT::freeMemory()
{
    if (!idToMotionMap_.empty())
    {
        for (auto &i : idToMotionMap_)
        {
            if (i->state_ != nullptr)
                si_->freeState(i->state_);
            delete i;
        }
        idToMotionMap_.clear();
    }
    delete LPAstarApx_;
    delete LPAstarLb_;
}

ompl::base::PlannerStatus ompl::geometric::LazyLBTRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    // update goal and check validity
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Goal undefined", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    while (const base::State *st = pis_.nextStart())
    {
        startMotion_ = createMotion(goal_s, st);
        break;
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    bool solved = false;

    auto *rmotion = new Motion(si_);
    base::State *xstate = si_->allocState();

    goalMotion_ = createGoalMotion(goal_s);

    CostEstimatorLb costEstimatorLb(goal, idToMotionMap_);
    LPAstarLb_ = new LPAstarLb(startMotion_->id_, goalMotion_->id_, graphLb_, costEstimatorLb);  // rooted at source
    CostEstimatorApx costEstimatorApx(this);
    LPAstarApx_ = new LPAstarApx(goalMotion_->id_, startMotion_->id_, graphApx_, costEstimatorApx);  // rooted at target
    double approxdif = std::numeric_limits<double>::infinity();
    // e+e/d.  K-nearest RRT*
    double k_rrg = boost::math::constants::e<double>() +
                   boost::math::constants::e<double>() / (double)si_->getStateSpace()->getDimension();

    ////////////////////////////////////////////
    // step (1) - RRT
    ////////////////////////////////////////////
    bestCost_ = std::numeric_limits<double>::infinity();
    rrt(ptc, goal_s, xstate, rmotion, approxdif);
    if (!ptc())
    {
        solved = true;

        ////////////////////////////////////////////
        // step (2) - Lazy construction of G_lb
        ////////////////////////////////////////////
        idToMotionMap_.push_back(goalMotion_);
        int k = getK(idToMotionMap_.size(), k_rrg);
        std::vector<Motion *> nnVec;
        nnVec.reserve(k);
        BOOST_FOREACH (Motion *motion, idToMotionMap_)
        {
            nn_->nearestK(motion, k, nnVec);
            BOOST_FOREACH (Motion *neighbor, nnVec)
                if (neighbor->id_ != motion->id_ && !edgeExistsLb(motion, neighbor))
                    addEdgeLb(motion, neighbor, distanceFunction(motion, neighbor));
        }
        idToMotionMap_.pop_back();
        closeBounds(ptc);
    }

    ////////////////////////////////////////////
    // step (3) - anytime planning
    ////////////////////////////////////////////
    while (!ptc())
    {
        std::tuple<Motion *, base::State *, double> res = rrtExtend(goal_s, xstate, rmotion, approxdif);
        Motion *nmotion = std::get<0>(res);
        base::State *dstate = std::get<1>(res);
        double d = std::get<2>(res);

        iterations_++;
        if (dstate != nullptr)
        {
            /* create a motion */
            Motion *motion = createMotion(goal_s, dstate);
            addEdgeApx(nmotion, motion, d);
            addEdgeLb(nmotion, motion, d);

            int k = getK(nn_->size(), k_rrg);
            std::vector<Motion *> nnVec;
            nnVec.reserve(k);
            nn_->nearestK(motion, k, nnVec);

            BOOST_FOREACH (Motion *neighbor, nnVec)
                if (neighbor->id_ != motion->id_ && !edgeExistsLb(motion, neighbor))
                    addEdgeLb(motion, neighbor, distanceFunction(motion, neighbor));

            closeBounds(ptc);
        }

        std::list<std::size_t> pathApx;
        double costApx = LPAstarApx_->computeShortestPath(pathApx);
        if (bestCost_ > costApx)
        {
            OMPL_INFORM("%s: approximation cost = %g", getName().c_str(), costApx);
            bestCost_ = costApx;
        }
    }

    if (solved)
    {
        std::list<std::size_t> pathApx;
        LPAstarApx_->computeShortestPath(pathApx);

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));

        // the path is in reverse order
        for (auto rit = pathApx.rbegin(); rit != pathApx.rend(); ++rit)
            path->append(idToMotionMap_[*rit]->state_);

        pdef_->addSolutionPath(path, !solved, 0);
    }

    si_->freeState(xstate);
    if (rmotion->state_ != nullptr)
        si_->freeState(rmotion->state_);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, !solved};
}

std::tuple<ompl::geometric::LazyLBTRRT::Motion *, ompl::base::State *, double> ompl::geometric::LazyLBTRRT::rrtExtend(
    const base::GoalSampleableRegion *goal_s, base::State *xstate, Motion *rmotion, double &approxdif)
{
    base::State *rstate = rmotion->state_;
    sampleBiased(goal_s, rstate);
    /* find closest state in the tree */
    Motion *nmotion = nn_->nearest(rmotion);
    base::State *dstate = rstate;

    /* find state to add */
    double d = distanceFunction(nmotion->state_, rstate);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state_, rstate, maxDistance_ / d, xstate);
        dstate = xstate;
        d = maxDistance_;
    }

    if (!checkMotion(nmotion->state_, dstate))
        return std::make_tuple((Motion *)nullptr, (base::State *)nullptr, 0.0);

    // motion is valid
    double dist = 0.0;
    bool sat = goal_s->isSatisfied(dstate, &dist);
    if (sat)
    {
        approxdif = dist;
    }
    if (dist < approxdif)
    {
        approxdif = dist;
    }

    return std::make_tuple(nmotion, dstate, d);
}

void ompl::geometric::LazyLBTRRT::rrt(const base::PlannerTerminationCondition &ptc, base::GoalSampleableRegion *goal_s,
                                      base::State *xstate, Motion *rmotion, double &approxdif)
{
    while (!ptc())
    {
        std::tuple<Motion *, base::State *, double> res = rrtExtend(goal_s, xstate, rmotion, approxdif);
        Motion *nmotion = std::get<0>(res);
        base::State *dstate = std::get<1>(res);
        double d = std::get<2>(res);

        iterations_++;
        if (dstate != nullptr)
        {
            /* create a motion */
            Motion *motion = createMotion(goal_s, dstate);
            addEdgeApx(nmotion, motion, d);

            if (motion == goalMotion_)
                return;
        }
    }
}

void ompl::geometric::LazyLBTRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state_));

    for (unsigned int i = 0; i < idToMotionMap_.size(); ++i)
    {
        const base::State *parent = idToMotionMap_[i]->state_;
        if (boost::in_degree(i, graphApx_) == 0)
            data.addGoalVertex(base::PlannerDataVertex(parent));
        if (boost::out_degree(i, graphApx_) == 0)
            data.addStartVertex(base::PlannerDataVertex(parent));
        else
        {
            boost::graph_traits<BoostGraph>::out_edge_iterator ei, ei_end;
            for (boost::tie(ei, ei_end) = boost::out_edges(i, graphApx_); ei != ei_end; ++ei)
            {
                std::size_t v = boost::target(*ei, graphApx_);
                data.addEdge(base::PlannerDataVertex(idToMotionMap_[v]->state_), base::PlannerDataVertex(parent));
            }
        }
    }
}

void ompl::geometric::LazyLBTRRT::sampleBiased(const base::GoalSampleableRegion *goal_s, base::State *rstate)
{
    /* sample random state (with goal biasing) */
    if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
        goal_s->sampleGoal(rstate);
    else
        sampler_->sampleUniform(rstate);
};

ompl::geometric::LazyLBTRRT::Motion *ompl::geometric::LazyLBTRRT::createMotion(const base::GoalSampleableRegion *goal_s,
                                                                               const base::State *st)
{
    if (goal_s->isSatisfied(st))
        return goalMotion_;

    auto *motion = new Motion(si_);
    si_->copyState(motion->state_, st);
    motion->id_ = idToMotionMap_.size();
    nn_->add(motion);
    idToMotionMap_.push_back(motion);
    addVertex(motion);

    return motion;
}

ompl::geometric::LazyLBTRRT::Motion *
ompl::geometric::LazyLBTRRT::createGoalMotion(const base::GoalSampleableRegion *goal_s)
{
    ompl::base::State *gstate = si_->allocState();
    goal_s->sampleGoal(gstate);

    auto *motion = new Motion(si_);
    motion->state_ = gstate;
    motion->id_ = idToMotionMap_.size();
    idToMotionMap_.push_back(motion);
    addVertex(motion);

    return motion;
}

void ompl::geometric::LazyLBTRRT::closeBounds(const base::PlannerTerminationCondition &ptc)
{
    std::list<std::size_t> pathApx;
    double costApx = LPAstarApx_->computeShortestPath(pathApx);
    std::list<std::size_t> pathLb;
    double costLb = LPAstarLb_->computeShortestPath(pathLb);

    while (costApx > (1. + epsilon_) * costLb)
    {
        if (ptc())
            return;

        auto pathLbIter = pathLb.end();
        pathLbIter--;
        std::size_t v = *pathLbIter;
        pathLbIter--;
        std::size_t u = *pathLbIter;

        while (edgeExistsApx(u, v))
        {
            v = u;
            --pathLbIter;
            u = *pathLbIter;
        }

        Motion *motionU = idToMotionMap_[u];
        Motion *motionV = idToMotionMap_[v];
        if (checkMotion(motionU, motionV))
        {
            // note that we change the direction between u and v due to the diff in definition between Apx and LB
            addEdgeApx(motionV, motionU,
                       distanceFunction(motionU, motionV));  // the distance here can be obtained from the LB graph
            pathApx.clear();
            costApx = LPAstarApx_->computeShortestPath(pathApx);
        }
        else  // the edge (u,v) was not collision free
        {
            removeEdgeLb(motionU, motionV);
            pathLb.clear();
            costLb = LPAstarLb_->computeShortestPath(pathLb);
        }
    }
}

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

#include "ompl/geometric/planners/gnat/GNAT.h"
#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <cassert>

ompl::geometric::GNAT::GNAT(const base::SpaceInformationPtr &si,
        bool useProjectedDistance,
        unsigned int degree, unsigned int minDegree,
        unsigned int maxDegree, unsigned int maxNumPtsPerLeaf, double estimatedDimension,
        unsigned int removedCacheSize) 
: base::Planner(si, "GNAT") 
{
    goalBias_ = 0.05;
    specs_.approximateSolutions = true;
    maxDistance_ = 0.0;
    setUseProjectedDistance(useProjectedDistance);
    setDegree(degree);
    setMinDegree(minDegree);
    setMaxDegree(maxDegree);
    setMaxNumPtsPerLeaf(maxNumPtsPerLeaf);
    setRemovedCacheSize(removedCacheSize);
    setEstimatedDimension(estimatedDimension);
    setMinValidPathFraction(0.2);
    tree_ = NULL;

    Planner::declareParam<double>("range", this, &GNAT::setRange, &GNAT::getRange);
    Planner::declareParam<double>("goal_bias", this, &GNAT::setGoalBias, &GNAT::getGoalBias);
    Planner::declareParam<bool>("use_projected_distance", this, &GNAT::setUseProjectedDistance, &GNAT::getUseProjectedDistance);
    Planner::declareParam<unsigned int>("degree", this, &GNAT::setDegree, &GNAT::getDegree);
    Planner::declareParam<unsigned int>("max_degree", this, &GNAT::setMaxDegree, &GNAT::getMaxDegree);
    Planner::declareParam<unsigned int>("min_degree", this, &GNAT::setMinDegree, &GNAT::getMinDegree);
    Planner::declareParam<unsigned int>("max_pts_per_leaf", this, &GNAT::setMaxNumPtsPerLeaf, &GNAT::getMaxNumPtsPerLeaf);
    Planner::declareParam<unsigned int>("removed_cache_size", this, &GNAT::setRemovedCacheSize, &GNAT::getRemovedCacheSize);
    Planner::declareParam<double>("estimated_dimension", this, &GNAT::setEstimatedDimension, &GNAT::getEstimatedDimension);
    Planner::declareParam<double>("min_valid_path_fraction", this, &GNAT::setMinValidPathFraction, &GNAT::getMinValidPathFraction);
}

ompl::geometric::GNAT::~GNAT(void)
{
    freeMemory();
    delete tree_;
    tree_ = NULL;
}

void ompl::geometric::GNAT::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    sc.configurePlannerRange(maxDistance_);
    setupTree();
}

void ompl::geometric::GNAT::setupTree(void)
{
    std::cout<<"GNAT Running with "<<degree_<<", "<<minDegree_<<", "<<maxDegree_<<", "<<maxNumPtsPerLeaf_<<", "<<estimatedDimension_<<", "<<removedCacheSize_<<std::endl;
    tree_ = new NearestNeighborsGNATSampler<Motion*>(degree_,minDegree_,maxDegree_,maxNumPtsPerLeaf_,estimatedDimension_,removedCacheSize_);
    if(useProjectedDistance_)
        tree_->setDistanceFunction(boost::bind(&GNAT::projectedDistanceFunction, this, _1, _2));
    else
        tree_->setDistanceFunction(boost::bind(&GNAT::distanceFunction, this, _1, _2));
} 


void ompl::geometric::GNAT::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    delete tree_;
    tree_ = NULL;
    setupTree();
}

void ompl::geometric::GNAT::freeMemory(void)
{
    if(tree_)
    {
        std::vector<Motion*> motions;
        tree_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

bool ompl::geometric::GNAT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                   *goal = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        addMotion(motion);
    }

    if (tree_->size() == 0)
    {
        msg_.error("There are no valid initial states!");
        return false;
    }

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();

    msg_.inform("Starting with %u states", tree_->size());

    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    base::State *xstate = si_->allocState();

    while (ptc() == false)
    {
        /* Decide on a state to expand from */
        Motion *existing = selectMotion();
        assert(existing);

        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(xstate);
        else
            if (!sampler_->sampleNear(xstate, existing->state, maxDistance_))
                continue;

        std::pair<base::State*, double> fail(xstate, 0.0);
        bool keep = si_->checkMotion(existing->state, xstate, fail);
        if (!keep && fail.second > minValidPathFraction_)
            keep = true;

        if (keep)
        {
            /* create a motion */
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, xstate);
            motion->parent = existing;

            addMotion(motion);
            double dist = 0.0;
            bool solved = goal->isSatisfied(motion->state, &dist);
            if (solved)
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
    }

    bool solved = false;
    bool approximate = false;
    if (solution == NULL)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != NULL)
    {
        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        PathGeometric *path = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->append(mpath[i]->state);
        goal->addSolutionPath(base::PathPtr(path), approximate, approxdif);
        solved = true;
    }

    si_->freeState(xstate);

    msg_.inform("Created %u states", tree_->size());

    return solved;
}

void ompl::geometric::GNAT::addMotion(Motion *motion)
{
    assert(tree_);
    tree_->add(motion);
}

ompl::geometric::GNAT::Motion* ompl::geometric::GNAT::selectMotion(void)
{
    assert(tree_);
    return tree_->sample();
}
void ompl::geometric::GNAT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    tree_->list(motions);
    for (std::vector<Motion*>::iterator it=motions.begin(); it!=motions.end(); it++)
        data.recordEdge((*it)->parent ? (*it)->parent->state : NULL, (*it)->state);
}

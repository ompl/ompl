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

#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"
#include <limits>

void ompl::geometric::RRT::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
}

void ompl::geometric::RRT::setup(void)
{
    Planner::setup(); 
    checkMotionLength(this, maxDistance_);

    if (!nn_)
	nn_.reset(new NearestNeighborsSqrtApprox<Motion*>());
    nn_->setDistanceFunction(boost::bind(&RRT::distanceFunction, this, _1, _2));
}

void ompl::geometric::RRT::freeMemory(void)
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

bool ompl::geometric::RRT::solve(const base::PlannerTerminationCondition &ptc)
{
    pis_.checkValidity();
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
    
    if (!goal)
    {
	msg_.error("Goal undefined");
	return false;
    }
    
    while (const base::State *st = pis_.nextStart())
    {
	Motion *motion = new Motion(si_);
	si_->copyState(motion->state, st);
	nn_->add(motion);
    }
    
    if (nn_->size() == 0)
    {
	msg_.error("There are no valid initial states!");
	return false;	
    }    

    if (!sampler_)
	sampler_ = si_->allocManifoldStateSampler();

    msg_.inform("Starting with %u states", nn_->size());
    
    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    Motion *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (ptc() == false)
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
	    si_->getStateManifold()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
	    dstate = xstate;
	}

	if (si_->checkMotion(nmotion->state, dstate))
	{
	    /* create a motion */
	    Motion *motion = new Motion(si_);
	    si_->copyState(motion->state, dstate);
	    motion->parent = nmotion;
	    
	    nn_->add(motion);
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
	    path->states.push_back(si_->cloneState(mpath[i]->state));
	goal->setDifference(approxdif);
	goal->setSolutionPath(base::PathPtr(path), approximate);

	if (approximate)
	    msg_.warn("Found approximate solution");
    }

    si_->freeState(xstate);
    if (rmotion->state)
	si_->freeState(rmotion->state);
    delete rmotion;
	
    msg_.inform("Created %u states", nn_->size());
    
    return goal->isAchieved();
}

void ompl::geometric::RRT::getPlannerData(base::PlannerData &data) const
{  
    Planner::getPlannerData(data);
    
    std::vector<Motion*> motions;
    if (nn_)
	nn_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	data.recordEdge(motions[i]->parent ? motions[i]->parent->state : NULL, motions[i]->state);
}

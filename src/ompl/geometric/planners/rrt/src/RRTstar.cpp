/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan Sucan */

#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <algorithm>
#include <limits>
#include <map>
#include <boost/math/constants/constants.hpp>

ompl::geometric::RRTstar::RRTstar(const base::SpaceInformationPtr &si) : base::Planner(si, "RRTstar")
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    delayCC_ = true;
    lastGoalMotion_ = NULL;

    iterations_ = 0;
    collisionChecks_ = 0;
    bestCost_ = 0.0 / 0.0;

    Planner::declareParam<double>("range", this, &RRTstar::setRange, &RRTstar::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTstar::setGoalBias, &RRTstar::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("delay_collision_checking", this, &RRTstar::setDelayCC, &RRTstar::getDelayCC, "0,1");

    Planner::addPlannerProgressFunction("iterations INTEGER",
                                        boost::bind(&RRTstar::getIterations, this));
    Planner::addPlannerProgressFunction("collision checks INTEGER",
                                        boost::bind(&RRTstar::getCollisionChecks, this));
    Planner::addPlannerProgressFunction("best cost REAL",
                                        boost::bind(&RRTstar::getBestCost, this));
}

ompl::geometric::RRTstar::~RRTstar(void)
{
    freeMemory();
}

void ompl::geometric::RRTstar::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&RRTstar::distanceFunction, this, _1, _2));
  

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_->hasOptimizationObjective())
	opt_ = pdef_->getOptimizationObjective();
    else
    {
        OMPL_INFORM("Defaulting to optimizing path length.");
        opt_.reset(new base::PathIntegralOptimizationObjective(si_));
    }
}

void ompl::geometric::RRTstar::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();

    lastGoalMotion_ = NULL;
    goalMotions_.clear();

    iterations_ = 0;
    collisionChecks_ = 0;
    bestCost_ = 0.0 / 0.0;
    
}

ompl::base::PlannerStatus ompl::geometric::RRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                  *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion  *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    bool symDist = si_->getStateSpace()->hasSymmetricDistance();
    bool symInterp = si_->getStateSpace()->hasSymmetricInterpolate();
    bool symCost = opt_->isSymmetric();

    if (!goal)
    {
        OMPL_ERROR("Goal undefined");
        return base::PlannerStatus::INVALID_GOAL;
    }

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_, opt_);
        si_->copyState(motion->state, st);
	opt_->getInitialCost(motion->state, motion->cost);
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

    Motion *solution       = lastGoalMotion_;
    base::Cost *bestCost   = NULL;
    Motion *approximation  = NULL;
    double approximatedist = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    Motion *rmotion        = new Motion(si_, opt_);
    base::State *rstate    = rmotion->state;
    base::State *xstate    = si_->allocState();

    // e+e/d.  K-nearest RRT*
    double k_rrg           = boost::math::constants::e<double>() + (boost::math::constants::e<double>()/(double)si_->getStateSpace()->getDimension());

    std::vector<Motion*>       nbh;
    std::vector<base::Cost*> costs;
    std::vector<base::Cost*>  incCosts;
    std::vector<unsigned> sortedCostIndices;
    std::vector<int>           valid;
    unsigned int               rewireTest = 0;
    unsigned int               statesGenerated = 0;

    if(solution)
        OMPL_INFORM("Starting with existing solution of cost %.5f", solution->cost);
    OMPL_INFORM("Initial k-nearest value of %u", (unsigned int)std::ceil(k_rrg * log(nn_->size()+1)));

    // more variables we'll need during rewiring
    base::Cost* nbhPrevCost = opt_->allocCost();
    base::Cost* nbhIncCost = opt_->allocCost();
    base::Cost* nbhNewCost = opt_->allocCost();
    
    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);

    // our planner progress data which we'll pass out to the user
    std::map<std::string, std::string> plannerProgressData;

    while (ptc == false)
    {
        iterations_++;
        // sample random state (with goal biasing)
        // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal states.
        if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);

        base::State *dstate = rstate;

        // find state to add to the tree
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

	++numCollisionChecks_;
        // Check if the motion between the nearest state and the state to add is valid
        ++collisionChecks_;
        if (si_->checkMotion(nmotion->state, dstate))
        {
            // create a motion
            Motion *motion = new Motion(si_, opt_);
            si_->copyState(motion->state, dstate);
	    motion->parent = nmotion;
      
            // This sounds crazy but for asymmetric distance functions this is necessary
            // For this case, it has to be FROM every other point TO our new point
            // NOTE THE ORDER OF THE boost::bind PARAMETERS
	    if (!symDist)
		nn_->setDistanceFunction(boost::bind(&RRTstar::distanceFunction, this, _1, _2));

            // Find nearby neighbors of the new motion - k-nearest RRT*
            unsigned int k = std::ceil(k_rrg * log(nn_->size()+1));
            nn_->nearestK(motion, k, nbh);
            rewireTest += nbh.size();
            statesGenerated++;

            // cache for distance computations
	    //
	    // Our cost caches only increase in size, so they're only
	    // resized if they can't fit the current neighborhood
	    if (costs.size() < nbh.size())
	    {
	        std::size_t prevSize = costs.size();
		costs.resize(nbh.size());
		incCosts.resize(nbh.size());
		sortedCostIndices.resize(nbh.size());
		for (std::size_t i = prevSize; i < nbh.size(); ++i)
		{
		    costs[i] = opt_->allocCost();
		    incCosts[i] = opt_->allocCost();
		}
	    }

            // cache for motion validity (only useful in a symmetric space)
	    //
	    // Our validity caches only increase in size, so they're
	    // only resized if they can't fit the current neighborhood
            if (symDist && symInterp)
            {
		if (valid.size() < nbh.size())
		    valid.resize(nbh.size());
                std::fill(valid.begin(), valid.begin()+nbh.size(), 0);
            }

            // Finding the nearest neighbor to connect to
            // By default, neighborhood states are sorted by cost, and collision checking
            // is performed in increasing order of cost
            if (delayCC_)
            {
                // calculate all costs and distances
	        for (std::size_t i = 0 ; i < nbh.size(); ++i)
                {
		    opt_->getIncrementalCost(nbh[i]->state, motion->state, incCosts[i]);
		    opt_->combineObjectiveCosts(nbh[i]->cost, incCosts[i], costs[i]);
                }

                // sort the nodes
                //
                // we're using index-value pairs so that we can get at
                // original, unsorted indices
		for (std::size_t i = 0; i < nbh.size(); ++i)
		    sortedCostIndices[i] = i;
                std::sort(sortedCostIndices.begin(), sortedCostIndices.begin()+nbh.size(), 
			  compareFn);

                // collision check until a valid motion is found
		for (std::vector<unsigned>::const_iterator i = sortedCostIndices.begin();
		     i != sortedCostIndices.begin()+nbh.size();
		     ++i)
		{
		    if (nbh[*i] == nmotion || si_->checkMotion(nbh[*i]->state, motion->state))
		    {
			if (nbh[*i] != nmotion)
			    ++collisionChecks_;
			opt_->copyCost(motion->incCost, incCosts[*i]);
			opt_->copyCost(motion->cost, costs[*i]);
			motion->parent = nbh[*i];
			if (symDist && symInterp)
			    valid[*i] = 1;
			break;
		    }
		    else if (symDist && symInterp)
			valid[*i] = -1;
		}
            }
            else // if not delayCC
            {
		opt_->getIncrementalCost(nmotion->state, motion->state, motion->incCost);
		opt_->combineObjectiveCosts(nmotion->cost, motion->incCost, motion->cost);
                // find which one we connect the new state to
	        for (std::size_t i = 0 ; i < nbh.size(); ++i)
                {
                    if (nbh[i] != nmotion)
                    {
                        opt_->getIncrementalCost(nbh[i]->state, motion->state, incCosts[i]);
                        opt_->combineObjectiveCosts(nbh[i]->cost,incCosts[i], costs[i]);
                        if (opt_->isCostLessThan(costs[i], motion->cost))
                        {
                            ++collisionChecks_;
                            if (si_->checkMotion(nbh[i]->state, motion->state))
                            {
                                opt_->copyCost(motion->incCost, incCosts[i]);
                                opt_->copyCost(motion->cost, costs[i]);
                                motion->parent = nbh[i];
                                if (symDist && symInterp)
                                    valid[i] = 1;
                            }
                            else if (symDist && symInterp)
                                valid[i] = -1;
                        }
                    }
                    else
                    {
                        opt_->copyCost(incCosts[i], motion->incCost);
                        opt_->copyCost(costs[i], motion->cost);
                        if (symDist && symInterp)
                            valid[i] = 1;
                    }
                }
            }

            // add motion to the tree
            nn_->add(motion);
            motion->parent->children.push_back(motion);

            bool checkForSolution = false;
            // rewire tree if needed
	    //
            // This sounds crazy but for asymmetric distance functions this is necessary
            // For this case, it has to be FROM our new point TO each other point
            // NOTE THE ORDER OF THE boost::bind PARAMETERS
            if (!symDist)
            {
                nn_->setDistanceFunction(boost::bind(&RRTstar::distanceFunction, this, _2, _1));
		nn_->nearestK(motion, k, nbh);
		rewireTest += nbh.size();
            }

            for (std::size_t i = 0 ; i < nbh.size(); ++i)
            {
		// In symmetric case, we use indices in
		// possibly-sorted costs list. In asymmetric case, we
		// just go through neighbors in default order
		std::size_t idx = (symDist && delayCC_) ? sortedCostIndices[i] : i;
                if (nbh[idx] != motion->parent)
                {
		    opt_->copyCost(nbhPrevCost, nbh[idx]->cost);
                    if (symDist && symCost)
                        opt_->copyCost(nbhIncCost, incCosts[idx]);
                    else
                        opt_->getIncrementalCost(motion->state, nbh[idx]->state, nbhIncCost);
		    opt_->combineObjectiveCosts(motion->cost, nbhIncCost, nbhNewCost);
                    if (opt_->isCostLessThan(nbhNewCost, nbhPrevCost))
                    {
			bool motionValid;
			if (symDist && symInterp)
			{
			    if (valid[idx] == 0)
			    {
				++collisionChecks_;
				motionValid = si_->checkMotion(motion->state, nbh[idx]->state);
			    }
			    else
				motionValid = (valid[idx] == 1);
			}
			else
			{
			    ++collisionChecks_;
			    motionValid = si_->checkMotion(motion->state, nbh[idx]->state);
			}
                        if (motionValid)
                        {
                            // Remove this node from its parent list
                            removeFromParent (nbh[idx]);

                            // Add this node to the new parent
                            nbh[idx]->parent = motion;
                            opt_->copyCost(nbh[idx]->incCost, nbhIncCost);
                            opt_->copyCost(nbh[idx]->cost, nbhNewCost);
                            nbh[idx]->parent->children.push_back(nbh[idx]);

                            // Update the costs of the node's children
                            updateChildCosts(nbh[idx]);

			    checkForSolution = true;
			}
		    }
                }
            }

            // Add the new motion to the goalMotion_ list, if it satisfies the goal
            double distanceFromGoal;
            if (goal->isSatisfied(motion->state, &distanceFromGoal))
            {
                goalMotions_.push_back(motion);
                checkForSolution = true;
            }

            // Checking for solution or iterative improvement
            for (size_t i = 0; i < goalMotions_.size() && checkForSolution; ++i)
            {

                if (!bestCost || opt_->isCostLessThan(goalMotions_[i]->cost, bestCost))
                {
                    bestCost = goalMotions_[i]->cost;
                    bestCost_ = opt_->getCostValue(bestCost);
                }

                sufficientlyShort = opt->isSatisfied(goalMotions_[i]->cost);
                if (sufficientlyShort)
                {
                    solution = goalMotions_[i];
                    break;
                }
                else if (!solution || 
			 opt_->isCostLessThan(goalMotions_[i]->cost,solution->cost))
                    solution = goalMotions_[i];
            }

            // Checking for approximate solution (closest state found to the goal)
            if (goalMotions_.size() == 0 && distanceFromGoal < approximatedist)
            {
                approximation = motion;
                approximatedist = distanceFromGoal;
            }
        }

        // terminate if a sufficient solution is found
        if (solution && sufficientlyShort)
            break;
    }

    bool approximate = (solution == 0);
    bool addedSolution = false;
    if (approximate)
        solution = approximation;
    else
        lastGoalMotion_ = solution;

    if (solution != 0)
    {
        // construct the solution path
        std::vector<Motion*> mpath;
        while (solution != 0)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // set the solution path
        PathGeometric *geoPath = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            geoPath->append(mpath[i]->state);

        base::PathPtr path(geoPath);
        // Add the solution path, whether it is approximate (not reaching the goal), and the
        // distance from the end of the path to the goal (-1 if satisfying the goal).
        base::PlannerSolution psol(path, approximate, approximate ? approximatedist : -1.0);
        // Does the solution satisfy the optimization objective?
        psol.optimized_ = sufficientlyShort;

        pdef_->addSolutionPath (psol);

        addedSolution = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    opt_->freeCost(rmotion->cost);
    opt_->freeCost(rmotion->incCost);
    delete rmotion;

    for (std::size_t i = 0; i < costs.size(); ++i)
	opt_->freeCost(costs[i]);
    for (std::size_t i = 0; i < incCosts.size(); ++i)
	opt_->freeCost(incCosts[i]);
    opt_->freeCost(nbhPrevCost);
    opt_->freeCost(nbhIncCost);
    opt_->freeCost(nbhNewCost);

    OMPL_INFORM("Created %u new states. Checked %lu rewire options. %u goal states in tree.", statesGenerated, rewireTest, goalMotions_.size());

    return base::PlannerStatus(addedSolution, approximate);
}

void ompl::geometric::RRTstar::removeFromParent(Motion *m)
{
    std::vector<Motion*>::iterator it = m->parent->children.begin ();
    while (it != m->parent->children.end ())
    {
        if (*it == m)
        {
            it = m->parent->children.erase(it);
            it = m->parent->children.end ();
        }
        else
            ++it;
    }
}

void ompl::geometric::RRTstar::updateChildCosts(Motion *m)
{
    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
        opt_->combineObjectiveCosts(m->cost, m->children[i]->incCost, m->children[i]->cost);
        updateChildCosts(m->children[i]);
    }
}

void ompl::geometric::RRTstar::freeMemory(void)
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (std::size_t i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
	    opt_->freeCost(motions[i]->cost);
	    opt_->freeCost(motions[i]->incCost);
            delete motions[i];
        }
    }
}

void ompl::geometric::RRTstar::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (std::size_t i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                         base::PlannerDataVertex(motions[i]->state));
    }
    data.properties["iterations INTEGER"] = boost::lexical_cast<std::string>(iterations_);
    data.properties["collision_checks INTEGER"] = 
	boost::lexical_cast<std::string>(numCollisionChecks_);
}

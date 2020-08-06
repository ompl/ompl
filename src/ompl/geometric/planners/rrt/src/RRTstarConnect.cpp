/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, University of Malaya
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

/* Author: Reza Mashayekhi */

#include "ompl/geometric/planners/rrt/RRTstarConnect.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/String.h"
#include "ompl/util/GeometricEquations.h"
#include "ompl/base/samplers/InformedStateSampler.h"

#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <vector>

ompl::geometric::RRTstarConnect::RRTstarConnect(const base::SpaceInformationPtr &si)
  : base::Planner(si, "RRTstarConnect")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RRTstarConnect::setRange, &RRTstarConnect::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("rewire_factor", this, &RRTstarConnect::setRewireFactor, &RRTstarConnect::getRewireFactor,
                                  "1.0:0.01:2.0");
    Planner::declareParam<bool>("use_k_nearest", this, &RRTstarConnect::setKNearest, &RRTstarConnect::getKNearest, "0,1");
    Planner::declareParam<bool>("delay_collision_checking", this, &RRTstarConnect::setDelayCC, &RRTstarConnect::getDelayCC, "0,1");

    Planner::declareParam<bool>("informed_sampling", this, &RRTstarConnect::setInformedSampling, &RRTstarConnect::getInformedSampling, "0,1");
    Planner::declareParam<bool>("tree_pruning", this, &RRTstarConnect::setTreePruning, &RRTstarConnect::getTreePruning, "0,1");

    Planner::declareParam<bool>("pruned_measure", this, &RRTstarConnect::setPrunedMeasure, &RRTstarConnect::getPrunedMeasure, "0,1");

    bestConnectionPoint_ = std::make_pair<Motion *, Motion *>(nullptr, nullptr);
    connectionPoints_.clear();
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();

    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

ompl::geometric::RRTstarConnect::~RRTstarConnect()
{
    freeMemory();
}

void ompl::geometric::RRTstarConnect::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                        "planning time.",
                        getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);

            // Store the new objective in the problem def'n
            pdef_->setOptimizationObjective(opt_);
        }

        // Set the bestCost_ and prunedCost_ as infinite
        bestCost_ = opt_->infiniteCost();
        prunedCost_ = opt_->infiniteCost();
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    // Calculate some constants:
    calculateRewiringLowerBounds();
}

void ompl::geometric::RRTstarConnect::freeMemory()
{
    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::RRTstarConnect::clear()
{
	setup_ = false;
    Planner::clear();
    sampler_.reset();
    infSampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    bestConnectionPoint_ = std::make_pair<Motion *, Motion *>(nullptr, nullptr);
    connectionPoints_.clear();
    costs.clear();
    incCosts.clear();
    sortedCostIndices.clear();
    valid.clear();
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedMeasure_ = 0.0;

    goalMotions_.clear();
    startMotions_.clear();
}

ompl::geometric::RRTstarConnect::GrowState ompl::geometric::RRTstarConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                             Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);

        /* Check if we have moved at all. Due to some stranger state spaces (e.g., the constrained state spaces),
         * interpolate can fail and no progress is made. Without this check, the algorithm gets stuck in a loop as it
         * thinks it is making progress, when none is actually occurring. */
        if (si_->equalStates(nmotion->state, tgi.xstate))
            return TRAPPED;

        dstate = tgi.xstate;
        reach = false;
    }

    bool validMotion = tgi.start ? si_->checkMotion(nmotion->state, dstate) :
                                   si_->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);

    if (!validMotion)
    {
        return TRAPPED;
    }

	Motion *motion = new Motion(si_);
	si_->copyState(motion->state, dstate);
	motion->parent = nmotion;
	motion->root = nmotion->root;
	motion->isConnectionPoint = false;
    motion->incCost = opt_->motionCost(nmotion->state, motion->state);
    motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

//	tgi.xmotion = motion;

    // Find nearby neighbors of the new motion
	std::vector<Motion *> nbh;

    getNeighbors(tree, motion, nbh);

    rewireTest += nbh.size();

    statesGenerated++;

    // cache for distance computations
    //
    // Our cost caches only increase in size, so they're only
    // resized if they can't fit the current neighborhood
    if (costs.size() < nbh.size())
    {
        costs.resize(nbh.size());
        incCosts.resize(nbh.size());
        sortedCostIndices.resize(nbh.size());
    }

    // cache for motion validity (only useful in a symmetric space)
    //
    // Our validity caches only increase in size, so they're
    // only resized if they can't fit the current neighborhood
    if (valid.size() < nbh.size())
        valid.resize(nbh.size());
    std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);

    // Finding the nearest neighbor to connect to
	// By default, neighborhood states are sorted by cost, and collision checking
	// is performed in increasing order of cost
	if (delayCC_)
	{
		// calculate all costs and distances
		for (std::size_t i = 0; i < nbh.size(); ++i)
		{
			incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
			costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
		}

		// sort the nodes
		//
		// we're using index-value pairs so that we can get at
		// original, unsorted indices
		for (std::size_t i = 0; i < nbh.size(); ++i)
			sortedCostIndices[i] = i;
		std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(), compareFn);

		// collision check until a valid motion is found
		//
		// ASYMMETRIC CASE: it's possible that none of these
		// neighbors are valid. This is fine, because motion
		// already has a connection to the tree through
		// nmotion (with populated cost fields!).
		for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
			 i != sortedCostIndices.begin() + nbh.size(); ++i)
		{
			if (nbh[*i] == nmotion ||
				((!useKNearest_ || si_->distance(nbh[*i]->state, motion->state) < maxDistance_) &&
						(tgi.start ? si_->checkMotion(nbh[*i]->state, motion->state) :
														si_->isValid(motion->state) && si_->checkMotion(motion->state, nbh[*i]->state))))
			{
				motion->incCost = incCosts[*i];
				motion->cost = costs[*i];
				motion->parent = nbh[*i];
				valid[*i] = 1;
				break;
			}
			else
				valid[*i] = -1;
		}
	}
	else  // if not delayCC
	{
		motion->incCost = opt_->motionCost(nmotion->state, motion->state);
		motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
		// find which one we connect the new state to
		for (std::size_t i = 0; i < nbh.size(); ++i)
		{
			if (nbh[i] != nmotion)
			{
				incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
				costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
				if (opt_->isCostBetterThan(costs[i], motion->cost))
				{
					if ((!useKNearest_ || si_->distance(nbh[i]->state, motion->state) < maxDistance_) &&
							(tgi.start ? si_->checkMotion(nbh[i]->state, motion->state) :
									si_->isValid(motion->state) && si_->checkMotion(motion->state, nbh[i]->state)))
					{
						motion->incCost = incCosts[i];
						motion->cost = costs[i];
						motion->parent = nbh[i];
						valid[i] = 1;
					}
					else
						valid[i] = -1;
				}
			}
			else
			{
				incCosts[i] = motion->incCost;
				costs[i] = motion->cost;
				valid[i] = 1;
			}
		}
	}

	tree->add(motion);
	tgi.xmotion = motion;
	motion->parent->children.push_back(motion);

	// update existing motions
	for (std::size_t i = 0; i < nbh.size(); ++i)
	{
		if (nbh[i] != motion->parent)
		{
			base::Cost nbhIncCost;
			if (opt_->isSymmetric())
				nbhIncCost = incCosts[i];
			else
				nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
			base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
			if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
			{
				if (valid[i] == 0)
				{
					validMotion = (!useKNearest_ || si_->distance(nbh[i]->state, motion->state) < maxDistance_) &&
							(tgi.start ? si_->checkMotion(motion->state, nbh[i]->state) :
									si_->isValid(motion->state) && si_->checkMotion(nbh[i]->state, motion->state));

				}
				else
				{
					validMotion = (valid[i] == 1);
				}

				if (validMotion)
				{
					// Remove this node from its parent list
					removeFromParent(nbh[i]);

					// Add this node to the new parent
					nbh[i]->parent = motion;
					nbh[i]->incCost = nbhIncCost;
					nbh[i]->cost = nbhNewCost;
					nbh[i]->parent->children.push_back(nbh[i]);

					// Update the costs of the node's children
					updateChildCosts(nbh[i]);

					checkForSolution = true;
				}
			}
		}
	}


    return reach ? REACHED : ADVANCED;
}

ompl::base::PlannerStatus ompl::geometric::RRTstarConnect::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
        startMotions_.push_back(motion);
    }

    // And assure that, if we're using an informed sampler, it's reset
    infSampler_.reset();

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Allocate a sampler if necessary
    if (!sampler_ && !infSampler_)
    {
        allocSampler();
    }

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    bool startTree = true;
    bool solved = false;

    rewireTest = 0;
    statesGenerated = 0;
    costs.clear();
    incCosts.clear();
    sortedCostIndices.clear();

    if (bestConnectionPoint_.first && bestConnectionPoint_.second)
        OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f", getName().c_str(), bestCost_);

    if (useKNearest_)
        OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(),
                    (unsigned int)std::ceil(k_rrt_ * log((double)(tStart_->size() + tGoal_->size() + 1u))));
    else
        OMPL_INFORM(
            "%s: Initial rewiring radius of %.2f", getName().c_str(),
            std::min(maxDistance_, r_rrt_ * std::pow(log((double)(tStart_->size() + tGoal_->size() + 1u)) / ((double)(tStart_->size() + tGoal_->size() + 1u)),
                                                     1 / (double)(si_->getStateDimension()))));
    while (!ptc)
    {
    	iterations_++;

        TreeData &tree = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);
                goalMotions_.push_back(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        /* sample random state */
        if (!sampleUniform(rmotion->state))
            continue;

        checkForSolution = false;
        GrowState gs = growTree(tree, tgi, rmotion);

        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need top copy again */
            if (gs != REACHED)
                si_->copyState(rmotion->state, tgi.xstate);

            GrowState gsc = ADVANCED;
            tgi.start = startTree;
            while (gsc == ADVANCED)
                gsc = growTree(otherTree, tgi, rmotion);

            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
            }

            Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
            Motion *goalMotion = startTree ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                startMotion->isConnectionPoint = true;
                goalMotion->isConnectionPoint = true;

                checkForSolution = true;

                connectionPoints_.push_back(std::pair<Motion *, Motion *>(startMotion, goalMotion));

                solved = true;
            }
            else
            {
            	if(!(bestConnectionPoint_.first && bestConnectionPoint_.second))
            		bestConnectionPoint_=std::pair<Motion *, Motion *>(startMotion, goalMotion);
                // We didn't reach the goal, but if we were extending the start
                // tree, then we can mark/improve the approximate path so far.
                if (!startTree)
                {
                    // We were working from the startTree.
                    double dist = 0.0;
                    goal->isSatisfied(tgi.xmotion->state, &dist);
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = tgi.xmotion;
                    }
                }
            }

            if (checkForSolution)
			{
				bool updatedSolution = false;
				if (!(bestConnectionPoint_.first && bestConnectionPoint_.second) && !connectionPoints_.empty())
				{
					// We have found our first solution, store it as the best. We only add one
					// connection point at a time, so there can only be one connection point at this moment.
					bestConnectionPoint_ = connectionPoints_.front();
					bestCost_ = opt_->combineCosts(bestConnectionPoint_.first->cost, bestConnectionPoint_.second->cost);
					updatedSolution = true;

					OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %u iterations (%u "
								"vertices in both trees(%u in Start Tree and %u in Goal Tree))",
								getName().c_str(), bestCost_, iterations_, tStart_->size() + tGoal_->size(), tStart_->size(), tGoal_->size());
				}
				else
				{
					// We already have a solution, iterate through the list of connection Points
					// and see if there's any improvement.
					for (auto &connectionPoint : connectionPoints_)
					{
						// Is this goal motion better than the (current) best?
						if ((opt_->combineCosts(connectionPoint.first->cost, connectionPoint.second->cost).value()) < bestCost_.value())
						{
							bestConnectionPoint_ = connectionPoint;
							bestCost_ = opt_->combineCosts(connectionPoint.first->cost, connectionPoint.second->cost);
							updatedSolution = true;

							// Check if it satisfies the optimization objective, if it does, break the for loop
							if (opt_->isSatisfied(bestCost_))
							{
								break;
							}
						}
					}
				}

				if (updatedSolution)
				{
					if (useTreePruning_)
					{
						pruneTrees(bestCost_);
					}
				}
			}
        }

        // terminate if a sufficient solution is found
        if (bestConnectionPoint_.first && bestConnectionPoint_.second && opt_->isSatisfied(bestCost_))
            break;
    }

    si_->freeState(tgi.xstate);
    delete rmotion;

    ptc.terminate();

    bool solutionFound = false;
    if (approxsol && !solved)
    {
    	solutionFound = true;

        Motion *solution = approxsol;
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));

        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);

        // Add the solution path.
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());

        // If we don't have a goal motion, the solution is approximate
        if (!(bestConnectionPoint_.first && bestConnectionPoint_.second))
            psol.setApproximate(approxdif);

        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, approxsol->cost, opt_->isSatisfied(bestCost_));


        pdef_->addSolutionPath(psol);
    }
    else if(bestConnectionPoint_.first && bestConnectionPoint_.second)
    {
    	solutionFound = true;
        Motion *solution = bestConnectionPoint_.first;
        std::vector<Motion *> mpath1;
        while (solution != nullptr)
        {
            mpath1.push_back(solution);
            solution = solution->parent;
        }

        solution = bestConnectionPoint_.second;
        std::vector<Motion *> mpath2;
        while (solution != nullptr)
        {
			mpath2.push_back(solution);
            solution = solution->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));
        path->getStates().reserve(mpath1.size() + mpath2.size());
        for (int i = mpath1.size() - 1; i >= 0; --i)
            path->append(mpath1[i]->state);
        for (auto &i : mpath2)
            path->append(i->state);

        // Add the solution path.
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());

        // If we don't have a goal motion, the solution is approximate
        if (!(bestConnectionPoint_.first && bestConnectionPoint_.second))
            psol.setApproximate(approxdif);

        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, opt_->combineCosts(bestConnectionPoint_.first->cost, bestConnectionPoint_.second->cost), opt_->isSatisfied(bestCost_));


        pdef_->addSolutionPath(psol);
    }


    OMPL_INFORM("%s: Created %u new states. All states in both trees %u (%u start + %u goal). \nChecked %u rewire options. %u connection points in graph. Final solution cost "
                "%.3f",
                getName().c_str(), statesGenerated, tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size(), rewireTest, connectionPoints_.size(), bestCost_);

	return base::PlannerStatus(solutionFound, bestConnectionPoint_.first== nullptr && bestConnectionPoint_.second== nullptr);
}

void ompl::geometric::RRTstarConnect::removeFromParent(Motion *m)
{
    for (auto it = m->parent->children.begin(); it != m->parent->children.end(); ++it)
    {
        if (*it == m)
        {
            m->parent->children.erase(it);
            break;
        }
    }
}

void ompl::geometric::RRTstarConnect::updateChildCosts(Motion *m)
{
    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
        updateChildCosts(m->children[i]);
    }
}

void ompl::geometric::RRTstarConnect::getNeighbors(TreeData &tree, Motion *motion, std::vector<Motion *> &nbh) const
{
    auto cardDbl = static_cast<double>(tree->size() + 1u);
    if (useKNearest_)
    {
        //- k-nearest RRT*
        unsigned int k = std::ceil(k_rrt_ * log(cardDbl));
        tree->nearestK(motion, k, nbh);
    }
    else
    {
        double r = std::min(
            maxDistance_, r_rrt_ * std::pow(log(cardDbl) / cardDbl, 1 / static_cast<double>(si_->getStateDimension())));
        tree->nearestR(motion, r, nbh);
    }
}

void ompl::geometric::RRTstarConnect::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    if(bestConnectionPoint_.first && bestConnectionPoint_.second)
    	data.addEdge(data.vertexIndex(bestConnectionPoint_.first->state), data.vertexIndex(bestConnectionPoint_.second->state));

    // Add some info.
    data.properties["approx goal distance REAL"] = ompl::toString(distanceBetweenTrees_);
}

int ompl::geometric::RRTstarConnect::pruneTrees(const base::Cost &pruneTreeCost)
{
    // Variable
    // The percent improvement (expressed as a [0,1] fraction) in cost
    double fracBetter;
    // The number pruned
    int numPruned = 0;

    if (opt_->isFinite(prunedCost_))
    {
        fracBetter = std::abs((pruneTreeCost.value() - prunedCost_.value()) / prunedCost_.value());
    }
    else
    {
        fracBetter = 1.0;
    }

    if (fracBetter > pruneThreshold_)
	{
		numPruned = pruneTree(tStart_, bestCost_, true);
		numPruned += pruneTree(tGoal_, bestCost_, false);
    }

    return numPruned;
}

int ompl::geometric::RRTstarConnect::pruneTree(TreeData &tree, const base::Cost &pruneTreeCost, bool isTreeStart)
{
	int numPruned = 0;
    // We are only pruning motions if they, AND all descendents, have a estimated cost greater than pruneTreeCost
    // The easiest way to do this is to find leaves that should be pruned and ascend up their ancestry until a
    // motion is found that is kept.
    // To avoid making an intermediate copy of the NN structure, we process the tree by descending down from the
    // start(s).
    // In the first pass, all Motions with a cost below pruneTreeCost, or Motion's with children with costs below
    // pruneTreeCost are added to the replacement NN structure,
    // while all other Motions are stored as either a 'leaf' or 'chain' Motion. After all the leaves are
    // disconnected and deleted, we check
    // if any of the the chain Motions are now leaves, and repeat that process until done.
    // This avoids (1) copying the NN structure into an intermediate variable and (2) the use of the expensive
    // NN::remove() method.

    // Variable
    // The queue of Motions to process:
    std::queue<Motion *, std::deque<Motion *>> motionQueue;
    // The list of leaves to prune
    std::queue<Motion *, std::deque<Motion *>> leavesToPrune;
    // The list of chain vertices to recheck after pruning
    std::list<Motion *> chainsToRecheck;

	tree->clear();

	if (isTreeStart)
	{

		// Put all the starts into the tStart_ structure and their children into the queue:
		// We do this so that start states are never pruned.
		for (auto &startMotion : startMotions_)
		{
			// Add to the tStart_
			tree->add(startMotion);

			// Add their children to the queue:
			addChildrenToList(&motionQueue, startMotion);
		}
	}
	else
	{
		// Put all the starts into the tGoal_ structure and their children into the queue:
		// We do this so that start states are never pruned.
		for (auto &goalMotion : goalMotions_)
		{
			// Add to the tGoal_
			tree->add(goalMotion);

			// Add their children to the queue:
			addChildrenToList(&motionQueue, goalMotion);
		}
	}

    while (motionQueue.empty() == false)
    {
        // Test, can the current motion ever provide a better solution?
        if (keepCondition(motionQueue.front(), pruneTreeCost, isTreeStart))
        {
            // Yes it can, so it definitely won't be pruned
            // Add it back into the NN structure
            tree->add(motionQueue.front());

            // Add it's children to the queue
            addChildrenToList(&motionQueue, motionQueue.front());
        }
        else
        {
            // No it can't, but does it have children?
            if (motionQueue.front()->children.empty() == false)
            {
                // Yes it does.
                // We can minimize the number of intermediate chain motions if we check their children
                // If any of them won't be pruned, then this motion won't either. This intuitively seems
                // like a nice balance between following the descendents forever.

                // Variable
                // Whether the children are definitely to be kept.
                bool keepAChild = false;

                // Find if any child is definitely not being pruned.
                for (unsigned int i = 0u; keepAChild == false && i < motionQueue.front()->children.size(); ++i)
                {
                    // Test if the child can ever provide a better solution
                    keepAChild = keepCondition(motionQueue.front()->children.at(i), pruneTreeCost, isTreeStart);
                }

                // Are we *definitely* keeping any of the children?
                if (keepAChild)
                {
                    // Yes, we are, so we are not pruning this motion
                    // Add it back into the NN structure.
                    tree->add(motionQueue.front());
                }
                else
                {
                    // No, we aren't. This doesn't mean we won't though
                    // Move this Motion to the temporary list
                    chainsToRecheck.push_back(motionQueue.front());
                }

                // Either way. add it's children to the queue
                addChildrenToList(&motionQueue, motionQueue.front());
            }
            else
            {
                // No, so we will be pruning this motion:
                leavesToPrune.push(motionQueue.front());
            }
        }

        // Pop the iterator, std::list::erase returns the next iterator
        motionQueue.pop();
    }

    // We now have a list of Motions to definitely remove, and a list of Motions to recheck
    // Iteratively check the two lists until there is nothing to to remove
    while (leavesToPrune.empty() == false)
    {
        // First empty the current leaves-to-prune
        while (leavesToPrune.empty() == false)
        {
            // If this leaf is a goal, remove it from the goal set
            if (leavesToPrune.front()->isConnectionPoint == true)
            {
            	if(isTreeStart)
            	{
					// Warn if pruning the _best_ goal
					if (leavesToPrune.front() == bestConnectionPoint_.first)
					{
						OMPL_ERROR("%s: Pruning the best connection point.", getName().c_str());
					}
					// Remove it
					// Find the connection point to remove
					for(auto cp : connectionPoints_)
					{
						if(cp.first == leavesToPrune.front())
						{
							cp.second->isConnectionPoint = false;
							connectionPoints_.erase(std::remove(connectionPoints_.begin(), connectionPoints_.end(), cp),connectionPoints_.end());
							break;
						}
					}
            	}
            	else
            	{
					// Warn if pruning the _best_ goal
					if (leavesToPrune.front() == bestConnectionPoint_.second)
					{
						OMPL_ERROR("%s: Pruning the best connection point.", getName().c_str());
					}
					// Remove it
					// Find the connection point to remove
					for(auto cp : connectionPoints_)
					{
						if(cp.second == leavesToPrune.front())
						{
							cp.first->isConnectionPoint = false;
							connectionPoints_.erase(std::remove(connectionPoints_.begin(), connectionPoints_.end(), cp),connectionPoints_.end());
							break;
						}
					}
            	}
            }

            // Remove the leaf from its parent
            removeFromParent(leavesToPrune.front());

            // Erase the actual motion
            // First free the state
            si_->freeState(leavesToPrune.front()->state);

            // then delete the pointer
            delete leavesToPrune.front();

            // And finally remove it from the list, erase returns the next iterator
            leavesToPrune.pop();

            // Update our counter
            ++numPruned;
        }

        // Now, we need to go through the list of chain vertices and see if any are now leaves
        auto mIter = chainsToRecheck.begin();
        while (mIter != chainsToRecheck.end())
        {
            // Is the Motion a leaf?
            if ((*mIter)->children.empty() == true)
            {
                // It is, add to the removal queue
                leavesToPrune.push(*mIter);

                // Remove from this queue, getting the next
                mIter = chainsToRecheck.erase(mIter);
            }
            else
            {
                // Is isn't, skip to the next
                ++mIter;
            }
        }
    }

    // Now finally add back any vertices left in chainsToReheck.
    // These are chain vertices that have descendents that we want to keep
    for (const auto &r : chainsToRecheck)
        // Add the motion back to the NN struct:
        tree->add(r);

    // All done pruning.
    // Update the cost at which we've pruned:
    prunedCost_ = pruneTreeCost;

    // And if we're using the pruned measure, the measure to which we've pruned
    if (usePrunedMeasure_)
    {
        prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);

        if (useKNearest_ == false)
        {
            calculateRewiringLowerBounds();
        }
    }
    // No else, prunedMeasure_ is the si_ measure by default.
    return numPruned;
}

void ompl::geometric::RRTstarConnect::addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion)
{
    for (auto &child : motion->children)
    {
        motionList->push(child);
    }
}

bool ompl::geometric::RRTstarConnect::keepCondition(const Motion *motion, const base::Cost &threshold, bool isTreeStart) const
{
    // We keep if the cost-to-come-heuristic of motion is <= threshold, by checking
    // if !(threshold < heuristic), as if b is not better than a, then a is better than, or equal to, b
    if (isTreeStart && bestConnectionPoint_.first && motion == bestConnectionPoint_.first)
	{
		// If the threshold is the theoretical minimum, bestConnectionPoint_ will sometimes fail the test due to floating point precision. Avoid that.
		return true;
	}
	else if (!isTreeStart && bestConnectionPoint_.second && motion == bestConnectionPoint_.second)
	{
		return true;
	}
    return !opt_->isCostBetterThan(threshold, solutionHeuristic(motion, isTreeStart));
}

ompl::base::Cost ompl::geometric::RRTstarConnect::solutionHeuristic(const Motion *motion, bool isTreeStart) const
{
	base::Cost costToCome = motion->cost;
	if (isTreeStart)
	{
		if (useAdmissibleCostToCome_)
		{
	        // Start with infinite cost
	        costToCome = opt_->infiniteCost();

	        // Find the min from each start
	        for (auto &startMotion : startMotions_)
	        {
	            costToCome = opt_->betterCost(
	                costToCome, opt_->motionCost(startMotion->state,
	                                             motion->state));  // lower-bounding cost from the start to the state
	        }
		}
		else
		{
			costToCome = motion->cost; // current cost from the state to the goal
		}
		const base::Cost costToGo =
				opt_->costToGo(motion->state, pdef_->getGoal().get()); // lower-bounding cost from the state to the goal
		return opt_->combineCosts(costToCome, costToGo);    // add the two costs
	}
	else
	{
		// The tree is goal tree
		if (useAdmissibleCostToCome_)
		{
			// Start with infinite cost
			costToCome = opt_->infiniteCost();

			// Find the min from each goal
			for (auto &goalMotion : goalMotions_)
			{
				costToCome = opt_->betterCost(costToCome,
						opt_->motionCost(motion->state, goalMotion->state)); // lower-bounding cost from the goal to the state
			}
		}
		else
		{
			costToCome = motion->cost; // current cost from the state to the goal
		}
		// Find the min from each start
		base::Cost costToGo = opt_->infiniteCost();
		for (auto &startMotion : startMotions_)
		{
			costToGo = opt_->betterCost(costToGo,
					opt_->motionCost(motion->state, startMotion->state)); // lower-bounding cost from the start to the state
		}
		return opt_->combineCosts(costToCome, costToGo);    // add the two costs
	}
}

void ompl::geometric::RRTstarConnect::setTreePruning(const bool prune)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // If we just disabled tree pruning, but we wee using prunedMeasure, we need to disable that as it required myself
    if (prune == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Store
    useTreePruning_ = prune;
}

void ompl::geometric::RRTstarConnect::setPrunedMeasure(bool informedMeasure)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option only works with informed sampling
    if (informedMeasure == true && (useInformedSampling_ == false || useTreePruning_ == false))
    {
        OMPL_ERROR("%s: InformedMeasure requires InformedSampling and TreePruning.", getName().c_str());
    }

    // Check if we're changed and update parameters if we have:
    if (informedMeasure != usePrunedMeasure_)
    {
        // Store the setting
        usePrunedMeasure_ = informedMeasure;

        // Update the prunedMeasure_ appropriately, if it has been configured.
        if (setup_ == true)
        {
            if (usePrunedMeasure_)
            {
                prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);
            }
            else
            {
                prunedMeasure_ = si_->getSpaceMeasure();
            }
        }

        // And either way, update the rewiring radius if necessary
        if (useKNearest_ == false)
        {
            calculateRewiringLowerBounds();
        }
    }
}

void ompl::geometric::RRTstarConnect::setInformedSampling(bool informedSampling)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // If we just disabled tree pruning, but we are using prunedMeasure, we need to disable that as it required myself
    if (informedSampling == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Check if we're changing the setting of informed sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (informedSampling != useInformedSampling_)
    {
        // If we're disabled informedSampling, and prunedMeasure is enabled, we need to disable that
        if (informedSampling == false && usePrunedMeasure_ == true)
        {
            setPrunedMeasure(false);
        }

        // Store the value
        useInformedSampling_ = informedSampling;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::RRTstarConnect::allocSampler()
{
    // Allocate the appropriate type of sampler.
    if (useInformedSampling_)
    {
        // We are using informed sampling, this can end-up reverting to rejection sampling in some cases
        OMPL_INFORM("%s: Using informed sampling.", getName().c_str());
        infSampler_ = opt_->allocInformedStateSampler(pdef_, numSampleAttempts_);
    }
    else
    {
        // We are using a regular sampler
        sampler_ = si_->allocStateSampler();
    }

    // No else
    // We are using a regular sampler
    sampler_ = si_->allocStateSampler();
}

bool ompl::geometric::RRTstarConnect::sampleUniform(base::State *statePtr)
{
    // Use the appropriate sampler
    if (useInformedSampling_)
    {
        // Attempt the focused sampler and return the result.
        // If bestCost is changing a lot by small amounts, this could
        // be prunedCost_ to reduce the number of times the informed sampling
        // transforms are recalculated.
        return infSampler_->sampleUniform(statePtr, bestCost_);
    }
    else
    {
        // Simply return a state from the regular sampler
        sampler_->sampleUniform(statePtr);

        // Always true
        return true;
    }
}

void ompl::geometric::RRTstarConnect::calculateRewiringLowerBounds()
{
    const auto dimDbl = static_cast<double>(si_->getStateDimension());

    // k_rrt > 2^(d + 1) * e * (1 + 1 / d).  K-nearest RRT*
    k_rrt_ = rewireFactor_ * (std::pow(2, dimDbl + 1) * boost::math::constants::e<double>() * (1.0 + 1.0 / dimDbl));

    // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
    // If we're not using the informed measure, prunedMeasure_ will be set to si_->getSpaceMeasure();
    r_rrt_ =
        rewireFactor_ *
        std::pow(2 * (1.0 + 1.0 / dimDbl) * (prunedMeasure_ / unitNBallMeasure(si_->getStateDimension())), 1.0 / dimDbl);
}

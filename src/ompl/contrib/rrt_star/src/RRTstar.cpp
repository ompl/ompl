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

/* Authors: Alejandro Perez, Sertac Karaman, Ioan Sucan */

#include "ompl/contrib/rrt_star/RRTstar.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <algorithm>
#include <limits>
#include <map>

ompl::geometric::RRTstar::RRTstar(const base::SpaceInformationPtr &si) : base::Planner(si, "RRTstar")
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    ballRadiusMax_ = 0.0;
    ballRadiusConst_ = 0.0;
    delayCC_ = true;

    Planner::declareParam<double>("range", this, &RRTstar::setRange, &RRTstar::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTstar::setGoalBias, &RRTstar::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("ball_radius_constant", this, &RRTstar::setBallRadiusConstant, &RRTstar::getBallRadiusConstant);
    Planner::declareParam<double>("max_ball_radius", this, &RRTstar::setMaxBallRadius, &RRTstar::getMaxBallRadius);
    Planner::declareParam<bool>("delay_collision_checking", this, &RRTstar::setDelayCC, &RRTstar::getDelayCC, "0,1");
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

    if (ballRadiusMax_ == 0.0)
        ballRadiusMax_ = si_->getMaximumExtent();
    if (ballRadiusConst_ == 0.0)
        ballRadiusConst_ = maxDistance_ * sqrt((double)si_->getStateSpace()->getDimension());

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&RRTstar::distanceFunction, this, _1, _2));
  

    // Setup optimization objective
    //
    // If a) no optimization objective was specified, or b) the
    // specified optimization objective is not a subclass of
    // BoundedAdditiveOptimizationObjective, then default to optimizing
    // path length as computed by the distance() function in the state
    // space.
    bool validObjective = true;				      
    if (pdef_->hasOptimizationObjective())
    {
        opt_ = boost::dynamic_pointer_cast<base::BoundedAdditiveOptimizationObjective>(pdef_->getOptimizationObjective());
        // If specified optimization objective not compatible (downcast didn't work)
        if (!opt_)
        {
            validObjective = false;
            OMPL_WARN("Optimization objective '%s' specified, but such an objective is not appropriate for %s. Only accumulative cost functions can be optimized.", pdef_->getOptimizationObjective()->getDescription().c_str(), getName().c_str());
        }
    }
    else // If no objective specified
        validObjective = false;

    // If we have no valid optimization objective, assume we're
    // optimizing path length
    if (!validObjective)
    {
        OMPL_INFORM("Defaulting to optimizing path length.");
        opt_.reset(new base::PathLengthOptimizationObjective(si_));
    }
}

void ompl::geometric::RRTstar::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
}

ompl::base::PlannerStatus ompl::geometric::RRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                  *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion  *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    bool symDist = si_->getStateSpace()->hasSymmetricDistance();

    if (!goal)
    {
        OMPL_ERROR("Goal undefined");
        return base::PlannerStatus::INVALID_GOAL;
    }

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

    Motion *solution       = NULL;
    Motion *approximation  = NULL;
    double approximatedist = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    Motion *rmotion     = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
    std::vector<Motion*> solCheck;
    std::vector<Motion*> nbh;
    std::vector<indexCostPair> costs;
    std::vector<double>  incCosts;
    std::vector<int>     valid;
    unsigned int         rewireTest = 0;
    double               stateSpaceDimensionConstant = 1.0 / (double)si_->getStateSpace()->getDimension();

    while (ptc == false)
    {
        // sample random state (with goal biasing)
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);

        base::State *dstate = rstate;

        // find state to add
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        if (si_->checkMotion(nmotion->state, dstate))
        {
            // create a motion
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            double incCost = opt_->getIncrementalCost(nmotion->state, dstate);
            motion->incCost = incCost;
            motion->cost = opt_->combineObjectiveCosts(nmotion->cost, incCost);

            // find nearby neighbors
            double r = std::min(ballRadiusConst_ * pow(log((double)(1 + nn_->size())) / (double)(nn_->size()), stateSpaceDimensionConstant),
                                ballRadiusMax_);
      
            // This sounds crazy but for asymmetric distance functions this is necessary
            // For this case, it has to be FROM every other point TO our new point
            // NOTE THE ORDER OF THE boost::bind PARAMETERS
            nn_->setDistanceFunction(boost::bind(&RRTstar::distanceFunction, this, _1, _2));
            nn_->nearestR(motion, r, nbh);

            // cache for distance computations
            costs.resize(nbh.size());
            incCosts.resize(nbh.size());
            // cache for motion validity
            if (symDist)
            {
                valid.resize(nbh.size());
                std::fill(valid.begin(), valid.end(), 0);
            }

            if(delayCC_)
            {
                // calculate all costs and distances
                for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                {
                    if (nbh[i] != nmotion)
                    {
                        incCosts[i] = opt_->getIncrementalCost(nbh[i]->state, dstate);	
                        costs[i].second = opt_->combineObjectiveCosts(nbh[i]->cost, incCosts[i]);
                    }
                    else
                    {
                        incCosts[i] = incCost;
                        costs[i].second = motion->cost;
                    }
                    costs[i].first = i;
                }

                // sort the nodes
                //
                // we're using index-value pairs so that we can get at
                // original, unsorted indices
                std::sort(costs.begin(), costs.end(), compareCost);

                // collision check until a valid motion is found
                for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                {
                    unsigned idx = costs[i].first;
                    if (nbh[idx] != nmotion)
                    {
                        if (costs[i].second < motion->cost)
                        {
                            if (si_->checkMotion(nbh[idx]->state, dstate))
                            {
                                motion->incCost = incCosts[idx];
                                motion->cost = costs[i].second;
                                motion->parent = nbh[idx];
                                if (symDist)
                                    valid[idx] = 1;
                                break;
                            }
                        }
                        else if (symDist)
                            valid[idx] = -1;
                    }
                    else
                    {
                        if (symDist)
                            valid[idx] = 1;
                        break;
                    }
                }
            }
            else
            {
                // find which one we connect the new state to
                for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                {
                    costs[i].first = i;
                    if (nbh[i] != nmotion)
                    {
                        incCosts[i] = opt_->getIncrementalCost(nbh[i]->state, dstate);
                        costs[i].second = opt_->combineObjectiveCosts(nbh[i]->cost,incCosts[i]);
                        if (costs[i].second < motion->cost)
                        {
                            if (si_->checkMotion(nbh[i]->state, dstate))
                            {
                                motion->incCost = incCosts[i];
                                motion->cost = costs[i].second;
                                motion->parent = nbh[i];
                                if (symDist)
                                    valid[i] = 1;
                            }
                            else if (symDist)
                                valid[i] = -1;
                        }
                    }
                    else
                    {
                        incCosts[i] = incCost;
                        costs[i].second = motion->cost;
                        if (symDist)
                            valid[i] = 1;
                    }
                }
            }

            // add motion to the tree
            nn_->add(motion);
            motion->parent->children.push_back(motion);

            solCheck.resize(1);
            solCheck[0] = motion;

            // rewire tree if needed
            // IN PREVIOUS CODE IT WAS ASSUMED THAT:
            // 1) cost = distance
            // 2) distance was a metric
            // 3) symmetric local steering
            //
            // Assumption 2 allowed for re-using previously computed
            // distances, but that just won't fly in general. This should be
            // optimized when we have a way of knowing whether our cost is
            // symmetric. Also, Assumption 3 allowed for using previously
            // computed collision check results stored in valid[]. This is
            // also not gonna work in general. Oh yeah and most importantly
            // without Assumption 2 we have to do ANOTHER NEAREST NEIGHBORS
            // SEARCH that goes FROM our new motion TO all other motions

            // This sounds crazy but for asymmetric distance functions this is necessary
            // For this case, it has to be FROM our new point TO each other point
            // NOTE THE ORDER OF THE boost::bind PARAMETERS
            if (!symDist)
            {
                nn_->setDistanceFunction(boost::bind(&RRTstar::distanceFunction, this, _2, _1));
                nn_->nearestR(motion, r, nbh);
            }
            rewireTest += nbh.size();
            for (unsigned int i = 0 ; i < nbh.size() ; ++i)
            {
                unsigned idx = symDist ? costs[i].first : i;
                // TODO: figure our what the ELSE to this actually means in an
                // asymmetric case            
                if (nbh[idx] != motion->parent)
                {
                    double nbhPrevCost = nbh[idx]->cost;
                    double incCost;
                    if (symDist)
                        incCost = incCosts[idx];
                    else
                        incCost = opt_->getIncrementalCost(motion->state, nbh[idx]->state);
                    double newCost = opt_->combineObjectiveCosts(motion->cost, incCost);
                    if (newCost < nbhPrevCost)
                    {
                        bool motionValid = symDist ?
                            (valid[idx] == 0 ? 
                                 si_->checkMotion(dstate, nbh[idx]->state) :
                                 valid[idx] == 1) :
                            si_->checkMotion(dstate, nbh[idx]->state);
                        if (motionValid)
                        {
                            // Remove this node from its parent list
                            removeFromParent (nbh[idx]);

                            // Add this node to the new parent
                            nbh[idx]->parent = motion;
                            nbh[idx]->incCost = incCost;
                            nbh[idx]->cost = newCost;
                            nbh[idx]->parent->children.push_back(nbh[idx]);
                            solCheck.push_back(nbh[idx]);

                            // Update the costs of the node's children
                            updateChildCosts(nbh[idx]);
                        }
                    }
                }
            }

            // Make sure to check the existing solution for improvement
            if (solution)
                solCheck.push_back(solution);

            // check if we found a solution
            for (unsigned int i = 0 ; i < solCheck.size() ; ++i)
            {
                double dist = 0.0;
                bool solved = goal->isSatisfied(solCheck[i]->state, &dist);
                sufficientlyShort = solved ? opt_->isSatisfied(solCheck[i]->cost) : false;

                if (solved)
                {
                    if (sufficientlyShort)
                    {
                        solution = solCheck[i];
                        break;
                    }
                    else if (!solution || (solCheck[i]->cost < solution->cost))
                    {
                        solution = solCheck[i];
                    }
                }
                else if (!solution && dist < approximatedist)
                {
                    approximation = solCheck[i];
                    approximatedist = dist;
                }
            }
        }

        // terminate if a sufficient solution is found
        if (solution && sufficientlyShort)
            break;
    }

    double solutionCost;
    bool approximate = (solution == NULL);
    bool addedSolution = false;
    if (approximate)
    {
        solution = approximation;
        solutionCost = approximatedist;
    }
    else
        solutionCost = solution->cost;

    if (solution != NULL)
    {
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
        pdef_->addSolutionPath(base::PathPtr(path), approximate, solutionCost);
        addedSolution = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("Created %u states. Checked %lu rewire options.", nn_->size(), rewireTest);

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
    for (size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost = opt_->combineObjectiveCosts(m->cost, m->children[i]->incCost);
        updateChildCosts(m->children[i]);
    }
}

void ompl::geometric::RRTstar::freeMemory(void)
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

void ompl::geometric::RRTstar::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
        data.addEdge (base::PlannerDataVertex (motions[i]->parent ? motions[i]->parent->state : NULL),
                      base::PlannerDataVertex (motions[i]->state));
}

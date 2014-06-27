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
/* CForest modifications authors: Javier V Gomez */

#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include <algorithm>
#include <limits>
#include <map>
#include <queue>
#include <boost/math/constants/constants.hpp>

#include <fstream>
#include "ompl/base/spaces/RealVectorStateSpace.h" // TO BE REMOVED

ompl::geometric::RRTstar::RRTstar(const base::SpaceInformationPtr &si) : base::Planner(si, "RRTstar")
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    delayCC_ = true;
    lastGoalMotion_ = NULL;
    
    pruneTreeCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    isCForest_ = false;
    pruneCostThreshold_ = 0.001;
    pruneStatesThreshold_ = 0.05;

    iterations_ = 0;
    collisionChecks_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    distanceDirection_ = FROM_NEIGHBORS;

    Planner::declareParam<double>("range", this, &RRTstar::setRange, &RRTstar::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTstar::setGoalBias, &RRTstar::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("delay_collision_checking", this, &RRTstar::setDelayCC, &RRTstar::getDelayCC, "0,1");
    Planner::declareParam<double>("prune_cost_threshold", this, &RRTstar::setPruneCostImprovementThreshold, &RRTstar::getPruneCostImprovementThreshold, "0.:.0001:1.");
    Planner::declareParam<double>("prune_states_threshold", this, &RRTstar::setPruneStatesImprovementThreshold, &RRTstar::getPruneStatesImprovementThreshold, "0.:.01:1.");
    
    addPlannerProgressProperty("iterations INTEGER",
                               boost::bind(&RRTstar::getIterationCount, this));
    addPlannerProgressProperty("collision checks INTEGER",
                               boost::bind(&RRTstar::getCollisionCheckCount, this));
    addPlannerProgressProperty("best cost REAL",
                               boost::bind(&RRTstar::getBestCost, this));
}

ompl::geometric::RRTstar::~RRTstar()
{
    freeMemory();
}

void ompl::geometric::RRTstar::setup()
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
        OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed planning time.", getName().c_str());
        opt_.reset(new base::PathLengthOptimizationObjective(si_));
    }
}

void ompl::geometric::RRTstar::clear()
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
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    pruneTreeCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}


// \TODO: This function is a bit inefficient in terms of memory allocation. We should free memory 
// only when states.size() < statesToInclude_.size(); otherwise, we allocate memory from 
// statesToInclude_.size() to states.size() -1 (allocState). Then we can use copyState() 
// instead of cloneState() and we save a lot of memory alloc / free.
void ompl::geometric::RRTstar::includeValidPath(const std::vector<const base::State *> &states, const base::Cost cost) 
{
    boost::mutex::scoped_lock slock(includePathsLock_);
    if (opt_->isCostBetterThan(cost, pruneTreeCost_))
    {
        si_->freeStates(statesToInclude_);
        statesToInclude_.clear();

        statesToInclude_.reserve(states.size());
        // TODO: would a cast to const state help? So statesToInclude could be vector <const base::State *>
        for (std::size_t i = 0; i < states.size(); ++i)
            statesToInclude_.push_back(si_->cloneState(states[i]));
            
        pruneTreeCost_ = cost;
        restartPrevMotion_ = true;
    }
}

ompl::base::PlannerStatus ompl::geometric::RRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                  *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion  *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    bool symDist = si_->getStateSpace()->hasSymmetricDistance();
    bool symInterp = si_->getStateSpace()->hasSymmetricInterpolate();
    bool symCost = opt_->isSymmetric();
    
    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->cost = opt_->identityCost();
        nn_->add(motion);
        startMotion_ = motion; // For CForest we assume there is only one initial state.
    }
    
    prevMotion_ = startMotion_;

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
    
    if (isCForest_ && pdef_->getStartStateCount() > 1)
    {
        OMPL_WARN("%s: There are more than 1 initial states. CForest is deactivated." , getName().c_str());
        isCForest_ = false;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution       = lastGoalMotion_;

    // \TODO Make this variable unnecessary, or at least have it
    // persist across solve runs
    base::Cost bestCost    = opt_->infiniteCost();
    bestCost_ = bestCost;
    pruneTreeCost_ = bestCost_;

    Motion *approximation  = NULL;
    double approximatedist = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    Motion *rmotion        = new Motion(si_);
    base::State *rstate    = rmotion->state;
    base::State *xstate    = si_->allocState();
 

    // e+e/d.  K-nearest RRT*
    double k_rrg           = boost::math::constants::e<double>() +
                             (boost::math::constants::e<double>() / (double)si_->getStateSpace()->getDimension());

    std::vector<Motion*>       nbh;

    std::vector<base::Cost>    costs;
    std::vector<base::Cost>    incCosts;
    std::vector<std::size_t>   sortedCostIndices;
    
    // CForest heuristics.
    base::Cost                 costToCome;
    base::Cost                 costToGo;
    base::Cost                 costTotal;

    std::vector<int>           valid;
    unsigned int               rewireTest = 0;
    unsigned int               statesGenerated = 0;

    base::Cost                 lastPruneCost = opt_->infiniteCost();

    pruneTreeCost_ = opt_->infiniteCost();

    if (solution)
        OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f", getName().c_str(), solution->cost.v);
    OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(), (unsigned int)std::ceil(k_rrg * log((double)(nn_->size()+1))));

    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);

    while (ptc == false)
    {
        iterations_++;

        // CFOREST Specific code: a new path has been shared by other thread.
        if (isCForest_)
        {
            addingSharedState_ = false;
            if (opt_->isCostBetterThan(pruneTreeCost_, bestCost_))
            {
                // \TODO: improve this! this should be done automatically when updating the tree, not hard-coded this way.
                // However I tried for days and I was unable to get other way working.
                for (std::size_t i = 0; i < goalMotions_.size(); ++i)
                    goalMotions_[i]->cost = pruneTreeCost_;

                // Only prune if the improvement is noticeable.
                if (1-pruneTreeCost_.v/lastPruneCost.v > pruneCostThreshold_ || std::isinf(lastPruneCost.v))
                {
                    lastPruneCost = bestCost_;
                    int n = pruneTree();
                    statesGenerated -= n;
                }
            }
        }

        if (isCForest_ && !statesToInclude_.empty()) 
        {
            boost::mutex::scoped_lock slock(includePathsLock_);
            si_->copyState(rmotion->state, statesToInclude_.back());
            si_->freeState(statesToInclude_.back());
            statesToInclude_.pop_back();
            addingSharedState_ = true;

            if (restartPrevMotion_)
            {
                prevMotion_ = startMotion_;
                restartPrevMotion_ = false;
            }
        }
        else 
        {
            // sample random state (with goal biasing)
            // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal states.
            if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ && goal_s->canSample())
                goal_s->sampleGoal(rstate);
            else
            {
                sampler_->sampleUniform(rstate);

                if (isCForest_)
                {
                    // Checking CForest condition.
                    costToCome = opt_->motionCost(pdef_->getStartState(0), rstate);
                    costToGo = base::goalRegionCostToGo(rstate, goal);
                    costTotal = opt_->combineCosts(costToCome, costToGo);
                    if (opt_->isCostBetterThan(pruneTreeCost_, costTotal))
                        continue;
                }
            }
        }

        if (!symDist)
            distanceDirection_ = FROM_NEIGHBORS;
        
        // \TODO: improve all this rstate, dstate, xstate mess. Probably we can get rid of some of them.
        base::State *dstate = rstate;
        Motion *nmotion = getInitialParent(rmotion, dstate, xstate);

        if (addingSharedState_ && si_->equalStates(nmotion->state, rstate))  // Duplicate states: ignore shared state.
            continue;

        // Check if the motion between the nearest state and the state to add is valid
        ++collisionChecks_;
        if (si_->checkMotion(nmotion->state, dstate))
        {
            // create a motion
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            motion->incCost = opt_->motionCost(nmotion->state, motion->state);
            motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

            // Find nearby neighbors of the new motion - k-nearest RRT*
            unsigned int k = std::ceil(k_rrg * log((double)(nn_->size() + 1)));
            nn_->nearestK(motion, k, nbh);
            rewireTest += nbh.size();
            ++statesGenerated;

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
                    incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                    costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
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
                //
                // ASYMMETRIC CASE: it's possible that none of these
                // neighbors are valid. This is fine, because motion
                // already has a connection to the tree through
                // nmotion (with populated cost fields!).
                for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                     i != sortedCostIndices.begin()+nbh.size();
                     ++i)
                {
                    if (nbh[*i] != nmotion)
                        ++collisionChecks_;
                    if (nbh[*i] == nmotion || si_->checkMotion(nbh[*i]->state, motion->state))
                    {
                        motion->incCost = incCosts[*i];
                        motion->cost = costs[*i];
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
                motion->incCost = opt_->motionCost(nmotion->state, motion->state);
                motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
                // find which one we connect the new state to
                for (std::size_t i = 0 ; i < nbh.size(); ++i)
                {
                    if (nbh[i] != nmotion)
                    {
                        incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                        costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                        if (opt_->isCostBetterThan(costs[i], motion->cost))
                        {
                            ++collisionChecks_;
                            if (si_->checkMotion(nbh[i]->state, motion->state))
                            {
                                motion->incCost = incCosts[i];
                                motion->cost = costs[i];
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
                        incCosts[i] = motion->incCost;
                        costs[i] = motion->cost;
                        if (symDist && symInterp)
                            valid[i] = 1;
                    }
                }
            }

            if (isCForest_ && !addingSharedState_)
            {
                costToCome = opt_->combineCosts(motion->parent->cost, opt_->motionCost(motion->parent->state, motion->state));
                costToGo = base::goalRegionCostToGo(motion->state, goal);
                costTotal = opt_->combineCosts(costToCome, costToGo);
                if (opt_->isCostBetterThan(costTotal, pruneTreeCost_))
                {
                    nn_->add(motion);
                    motion->parent->children.push_back(motion);
                }
                else // If the new motion does not improve the pruneTreeCost_ it is ignored.
                {
                    --statesGenerated;
                    si_->freeState(motion->state);
                    delete motion;
                    continue;
                }
            }
            else
            {
                nn_->add(motion);
                motion->parent->children.push_back(motion);
                prevMotion_ = motion; // For CForest only.
            }

            bool checkForSolution = false;
            // rewire tree if needed
            //
            // Set directionality of distance function to be FROM new
            // state TO neighbors, since this is how the routing
            // should occur in tree rewiring
            if (!symDist)
            {
                distanceDirection_ = TO_NEIGHBORS;
                nn_->nearestK(motion, k, nbh);
                rewireTest += nbh.size();
            }

            for (std::size_t i = 0; i < nbh.size(); ++i)
            {
                if (nbh[i] != motion->parent)
                {
                    base::Cost nbhIncCost;
                    if (symDist && symCost)
                        nbhIncCost = incCosts[i];
                    else
                        nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
                    base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
                    if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
                    {
                        bool motionValid;
                        if (symDist && symInterp)
                        {
                            if (valid[i] == 0)
                            {
                                ++collisionChecks_;
                                motionValid = si_->checkMotion(motion->state, nbh[i]->state);
                            }
                            else
                                motionValid = (valid[i] == 1);

                        }
                        else
                        {
                            ++collisionChecks_;
                            motionValid = si_->checkMotion(motion->state, nbh[i]->state);
                        }
                        if (motionValid)
                        {
                            // Remove this node from its parent list
                            removeFromParent (nbh[i]);

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

            // Add the new motion to the goalMotion_ list, if it satisfies the goal
            double distanceFromGoal;
            if (goal->isSatisfied(motion->state, &distanceFromGoal))
            {
                goalMotions_.push_back(motion);
                checkForSolution = true;
            }

            // Checking for solution or iterative improvement
            if (checkForSolution)
            {
                bool updatedSolution = false;
                for (size_t i = 0; i < goalMotions_.size(); ++i)
                {
                    if (opt_->isCostBetterThan(goalMotions_[i]->cost, bestCost))
                    {
                        bestCost = goalMotions_[i]->cost;
                        bestCost_ = bestCost;
                        updatedSolution = true;
                    }

                    sufficientlyShort = opt_->isSatisfied(goalMotions_[i]->cost);
                    if (sufficientlyShort)
                     {
                         solution = goalMotions_[i];
                         break;
                     }
                     else if (!solution ||
                         opt_->isCostBetterThan(goalMotions_[i]->cost,solution->cost)) 
                    {
                        solution = goalMotions_[i];
                        updatedSolution = true;
                    }
                }

                // CFOREST sharing path only when there are not more shared states to include.
                //if (isCForest_ && updatedSolution && statesToInclude_.empty()) 
                if (isCForest_ && updatedSolution ) 
                {
                    if (opt_->isCostBetterThan(bestCost_, pruneTreeCost_))
                    {
                        pruneTreeCost_ = bestCost_;
                        // Only prune if the improvement is noticeable.
                        // \TODO: would work if we substitute isinf() for == opt_->infiniteCost() ?
                        if (1-pruneTreeCost_.v/lastPruneCost.v > pruneCostThreshold_ || std::isinf(lastPruneCost.v))
                        {
                            lastPruneCost = bestCost_;
                            int n = pruneTree();
                            statesGenerated -= n;
                        }

                        std::vector<const base::State *> spath;
                        Motion *intermediate_solution = solution->parent; // Do not include goal state to simplify code.

                        do
                        {
                            spath.push_back(intermediate_solution->state);
                            intermediate_solution = intermediate_solution->parent;
                        } while (intermediate_solution->parent != 0); // Do not include the start state.

                        pdef_->getIntermediateSolutionCallback()(this, spath, bestCost_);
                    }
                }
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
        base::PlannerSolution psol(path, approximate, approximate ? approximatedist : -1.0, getName());
        // Does the solution satisfy the optimization objective?
        psol.optimized_ = sufficientlyShort;

        pdef_->addSolutionPath (psol);

        addedSolution = true;
    }
    
    int n = pruneTree();
    statesGenerated -= n;
    detelePrunedMotions();
    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree.", getName().c_str(), statesGenerated, rewireTest, goalMotions_.size());

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
        m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
        updateChildCosts(m->children[i]);
    }
}

void ompl::geometric::RRTstar::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (std::size_t i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
    
    si_->freeStates(statesToInclude_);
    statesToInclude_.clear();

    detelePrunedMotions();
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
        boost::lexical_cast<std::string>(collisionChecks_);
}

std::string ompl::geometric::RRTstar::getIterationCount() const
{
  return boost::lexical_cast<std::string>(iterations_);
}
std::string ompl::geometric::RRTstar::getCollisionCheckCount() const
{
  return boost::lexical_cast<std::string>(collisionChecks_);
}
std::string ompl::geometric::RRTstar::getBestCost() const
{
  return boost::lexical_cast<std::string>(bestCost_.v);
}

///////////////////////////////////////
// Javi added code.
void ompl::geometric::RRTstar::saveTree(const char * filename) 
{
	 OMPL_INFORM("Saving into %s", filename);
	 
	 std::vector<Motion*> tree;
	 nn_->list(tree);
	 
	 std::fstream fs;
	 fs.open (filename, std::fstream::out | std::fstream::trunc);
	 
	 fs << tree.size() << std::endl;
	 
	 // First node has no parent.
	 fs << tree[0]->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]	<< "\t"
		<< tree[0]->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << std::endl;
	
	 for (size_t i = 1; i < tree.size(); ++i) 
	 {
		 fs << tree[i]->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]	<< "\t"
		    << tree[i]->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << "\t"
			<< tree[i]->parent->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] << "\t"
		    << tree[i]->parent->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << std::endl; 
	 }
	 
	 fs.close();
}

int ompl::geometric::RRTstar::pruneTree()
{
    base::Cost costToGo;
    base::Cost costToCome;
    base::Cost totalCost;
    std::vector<Motion*> tree, newTree, toBePruned;
    tree.reserve(nn_->size()); 
    newTree.reserve(nn_->size());
    toBePruned.reserve(nn_->size());
    nn_->list(tree);

    base::Goal *goal = pdef_->getGoal().get();

    Motion *candidate;
    std::queue<Motion*> candidates;
    
    candidates.push(getRootMotion(tree[0]));

    while (!candidates.empty())
    {
        candidate = candidates.front();
        candidates.pop();

        costToCome = opt_->motionCost(pdef_->getStartState(0), candidate->state);
        costToGo = base::goalRegionCostToGo(candidate->state, goal); // h_g
        totalCost = opt_->combineCosts(costToCome, costToGo); // h_s + h_g

        if ( opt_->isCostBetterThan(totalCost, pruneTreeCost_))
        {
            newTree.push_back(candidate);
            for(std::size_t i = 0; i < candidate->children.size(); ++i)
                candidates.push(candidate->children[i]);
        }
        else {
            toBePruned.push_back(candidate);
        }
    }

    // To create the new nn takes one order of magnitude in time more than just checking how many 
    // states would be pruned. Therefore, only prune if it removes a significant amount of states.
    if (1. - ((double)newTree.size()) / tree.size() > pruneStatesThreshold_)
    {
        // TODO: the deletion of the pruned nodes can be improved. However, it requires to be really
        // careful when dealing with pruneTreeCost_ and goal motions, since they could be pruned if pruneTreeCost_
        // is updated while pruning. This does not avoid it, but does not delete the goal motion so it can be
        // removed from the tree but still used. It is kept this way in order to maintain the code understandable.
        for (std::size_t i = 0; i < toBePruned.size(); ++i)
        {
            removeFromParent(toBePruned[i]);
            toBeDeleted_.push_back(toBePruned[i]);
        }

        nn_->clear();
        nn_->add(newTree);

        return (tree.size() - newTree.size());
    }
    return 0;
}

void ompl::geometric::RRTstar::detelePrunedMotions()
{
    while (!toBeDeleted_.empty())
    {
        Motion *mto_delete = toBeDeleted_.front();
        toBeDeleted_.pop_front();

        for(std::size_t i = 0; i < mto_delete->children.size(); ++i) 
            toBeDeleted_.push_back(mto_delete->children[i]);

        si_->freeState(mto_delete->state);
        delete mto_delete;
    }
}

ompl::geometric::RRTstar::Motion* ompl::geometric::RRTstar::getRootMotion(ompl::geometric::RRTstar::Motion *seed)
{
    while (seed->parent != 0)
        seed = seed->parent;

    return seed;
}

ompl::geometric::RRTstar::Motion* ompl::geometric::RRTstar::getInitialParent(ompl::geometric::RRTstar::Motion *rmotion, base::State *dstate, base::State *xstate)
{
    // find closest state in the tree
    Motion *nmotion = nn_->nearest(rmotion);
    if (addingSharedState_)
    {
        if (si_->equalStates(rmotion->state, nmotion->state))
            prevMotion_ = nmotion;
        else
            nmotion = prevMotion_;
    }
    else
    {
        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);
        double d = si_->distance(nmotion->state, rmotion->state);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, dstate);
            dstate = xstate;
        }
    }

    return nmotion;
}

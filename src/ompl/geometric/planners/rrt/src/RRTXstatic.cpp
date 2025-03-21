/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Georgia Institute of Technology
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

/* Author: Florian Hauer */

#include "ompl/geometric/planners/rrt/RRTXstatic.h"
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include "ompl/base/Goal.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/GeometricEquations.h"

ompl::geometric::RRTXstatic::RRTXstatic(const base::SpaceInformationPtr &si)
  : base::Planner(si, "RRTXstatic")
  , mc_(opt_, pdef_)
  , q_(mc_)
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.canReportIntermediateSolutions = true;

    Planner::declareParam<double>("range", this, &RRTXstatic::setRange, &RRTXstatic::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTXstatic::setGoalBias, &RRTXstatic::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("epsilon", this, &RRTXstatic::setEpsilon, &RRTXstatic::getEpsilon, "0.:.01:10.");
    Planner::declareParam<double>("rewire_factor", this, &RRTXstatic::setRewireFactor, &RRTXstatic::getRewireFactor,
                                  "1.0:0.01:2."
                                  "0");
    Planner::declareParam<bool>("use_k_nearest", this, &RRTXstatic::setKNearest, &RRTXstatic::getKNearest, "0,1");
    Planner::declareParam<bool>("update_children", this, &RRTXstatic::setUpdateChildren, &RRTXstatic::getUpdateChildren,
                                "0,1");
    Planner::declareParam<int>("rejection_variant", this, &RRTXstatic::setVariant, &RRTXstatic::getVariant, "0:3");
    Planner::declareParam<double>("rejection_variant_alpha", this, &RRTXstatic::setAlpha, &RRTXstatic::getAlpha, "0.:"
                                                                                                                 "1.");
    Planner::declareParam<bool>("informed_sampling", this, &RRTXstatic::setInformedSampling,
                                &RRTXstatic::getInformedSampling, "0,"
                                                                  "1");
    Planner::declareParam<bool>("sample_rejection", this, &RRTXstatic::setSampleRejection,
                                &RRTXstatic::getSampleRejection, "0,1");
    Planner::declareParam<unsigned int>("number_sampling_attempts", this, &RRTXstatic::setNumSamplingAttempts,
                                &RRTXstatic::getNumSamplingAttempts, "10:10:100000");

    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
    addPlannerProgressProperty("motions INTEGER", [this] { return numMotionsProperty(); });
    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

ompl::geometric::RRTXstatic::~RRTXstatic()
{
    freeMemory();
}

void ompl::geometric::RRTXstatic::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    if (!si_->getStateSpace()->hasSymmetricDistance() || !si_->getStateSpace()->hasSymmetricInterpolate())
    {
        OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.", getName().c_str());
    }

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

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
        // Set the bestCost_ as infinite
        bestCost_ = opt_->infiniteCost();
        mc_ = MotionCompare(opt_, pdef_);
        q_ = BinaryHeap<Motion *, MotionCompare>(mc_);
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    // Calculate some constants:
    calculateRewiringLowerBounds();
}

void ompl::geometric::RRTXstatic::clear()
{
    setup_ = false;
    Planner::clear();
    sampler_.reset();
    infSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();

    lastGoalMotion_ = nullptr;
    goalMotions_.clear();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

ompl::base::PlannerStatus ompl::geometric::RRTXstatic::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // Check if there are more starts
    if (pis_.haveMoreStartStates() == true)
    {
        // There are, add them
        while (const base::State *st = pis_.nextStart())
        {
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, st);
            motion->cost = opt_->identityCost();
            nn_->add(motion);
        }

        // And assure that, if we're using an informed sampler, it's reset
        infSampler_.reset();
    }
    // No else

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Allocate a sampler if necessary
    if (!sampler_ && !infSampler_)
    {
        allocSampler();
    }

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    if (!si_->getStateSpace()->isMetricSpace())
        OMPL_WARN("%s: The state space (%s) is not metric and as a result the optimization objective may not satisfy "
                  "the triangle inequality. "
                  "You may need to disable rejection.",
                  getName().c_str(), si_->getStateSpace()->getName().c_str());

    const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

    Motion *solution = lastGoalMotion_;

    Motion *approximation = nullptr;
    double approximatedist = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
    base::State *dstate;

    Motion *motion;
    Motion *nmotion;
    Motion *nb;
    Motion *min;
    Motion *c;
    bool feas;

    unsigned int rewireTest = 0;
    unsigned int statesGenerated = 0;

    base::Cost incCost, cost;

    if (solution)
        OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f", getName().c_str(),
                    solution->cost.value());

    if (useKNearest_)
        OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(),
                    (unsigned int)std::ceil(k_rrt_ * log((double)(nn_->size() + 1u))));
    else
        OMPL_INFORM(
            "%s: Initial rewiring radius of %.2f", getName().c_str(),
            std::min(maxDistance_, r_rrt_ * std::pow(log((double)(nn_->size() + 1u)) / ((double)(nn_->size() + 1u)),
                                                     1 / (double)(si_->getStateDimension()))));

    while (ptc == false)
    {
        iterations_++;

        // Computes the RRG values for this iteration (number or radius of neighbors)
        calculateRRG();

        // sample random state (with goal biasing)
        // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal
        // states.
        if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ &&
            goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
        {
            // Attempt to generate a sample, if we fail (e.g., too many rejection attempts), skip the remainder of this
            // loop and return to try again
            if (!sampleUniform(rstate))
                continue;
        }

        // find closest state in the tree
        nmotion = nn_->nearest(rmotion);

        if (intermediateSolutionCallback && si_->equalStates(nmotion->state, rstate))
            continue;

        dstate = rstate;

        // find state to add to the tree
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        // Check if the motion between the nearest state and the state to add is valid
        if (si_->checkMotion(nmotion->state, dstate))
        {
            // create a motion
            motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            incCost = opt_->motionCost(nmotion->state, motion->state);
            motion->cost = opt_->combineCosts(nmotion->cost, incCost);

            // Find nearby neighbors of the new motion
            getNeighbors(motion);

            // find which one we connect the new state to
            for (auto it = motion->nbh.begin(); it != motion->nbh.end();)
            {
                nb = it->first;
                feas = it->second;

                // Compute cost using nb as a parent
                incCost = opt_->motionCost(nb->state, motion->state);
                cost = opt_->combineCosts(nb->cost, incCost);
                if (opt_->isCostBetterThan(cost, motion->cost))
                {
                    // Check range and feasibility
                    if ((!useKNearest_ || distanceFunction(motion, nb) < maxDistance_) &&
                        si_->checkMotion(nb->state, motion->state))
                    {
                        // mark than the motino has been checked as valid
                        it->second = true;

                        motion->cost = cost;
                        motion->parent = nb;
                        ++it;
                    }
                    else
                    {
                        // Remove unfeasible neighbor from the list of neighbors
                        it = motion->nbh.erase(it);
                    }
                }
                else
                {
                    ++it;
                }
            }

            // Check if the vertex should included
            if (!includeVertex(motion))
            {
                si_->freeState(motion->state);
                delete motion;
                continue;
            }

            // Update neighbor motions neighbor datastructure
            for (auto it = motion->nbh.begin(); it != motion->nbh.end(); ++it)
            {
                it->first->nbh.emplace_back(motion, it->second);
            }

            // add motion to the tree
            ++statesGenerated;
            nn_->add(motion);
            if (updateChildren_)
                motion->parent->children.push_back(motion);

            // add the new motion to the queue to propagate the changes
            updateQueue(motion);

            bool checkForSolution = false;

            // Add the new motion to the goalMotion_ list, if it satisfies the goal
            double distanceFromGoal;
            if (goal->isSatisfied(motion->state, &distanceFromGoal))
            {
                goalMotions_.push_back(motion);
                checkForSolution = true;
            }

            // Process the elements in the queue and rewire the tree until epsilon-optimality
            while (!q_.empty())
            {
                // Get element to update
                min = q_.top()->data;
                // Remove element from the queue and NULL the handle so that we know it's not in the queue anymore
                q_.pop();
                min->handle = nullptr;

                // Stop cost propagation if it is not in the relevant region
                if (opt_->isCostBetterThan(bestCost_, mc_.costPlusHeuristic(min)))
                    break;

                // Try min as a parent to optimize each neighbor
                for (auto it = min->nbh.begin(); it != min->nbh.end();)
                {
                    nb = it->first;
                    feas = it->second;

                    // Neighbor culling: removes neighbors farther than the neighbor radius
                    if ((!useKNearest_ || min->nbh.size() > rrg_k_) && distanceFunction(min, nb) > rrg_r_)
                    {
                        it = min->nbh.erase(it);
                        continue;
                    }

                    // Calculate cost of nb using min as a parent
                    incCost = opt_->motionCost(min->state, nb->state);
                    cost = opt_->combineCosts(min->cost, incCost);

                    // If cost improvement is better than epsilon
                    if (opt_->isCostBetterThan(opt_->combineCosts(cost, epsilonCost_), nb->cost))
                    {
                        if (nb->parent != min)
                        {
                            // changing parent, check feasibility
                            if (!feas)
                            {
                                feas = si_->checkMotion(nb->state, min->state);
                                if (!feas)
                                {
                                    // Remove unfeasible neighbor from the list of neighbors
                                    it = min->nbh.erase(it);
                                    continue;
                                }
                                else
                                {
                                    // mark than the motino has been checked as valid
                                    it->second = true;
                                }
                            }
                            if (updateChildren_)
                            {
                                // Remove this node from its parent list
                                removeFromParent(nb);
                                // add it as a children of min
                                min->children.push_back(nb);
                            }
                            // Add this node to the new parent
                            nb->parent = min;
                            ++rewireTest;
                        }
                        nb->cost = cost;

                        // Add to the queue for more improvements
                        updateQueue(nb);

                        checkForSolution = true;
                    }
                    ++it;
                }
                if (updateChildren_)
                {
                    // Propagatino of the cost to the children
                    for (auto it = min->children.begin(), end = min->children.end(); it != end; ++it)
                    {
                        c = *it;
                        incCost = opt_->motionCost(min->state, c->state);
                        cost = opt_->combineCosts(min->cost, incCost);
                        c->cost = cost;
                        // Add to the queue for more improvements
                        updateQueue(c);

                        checkForSolution = true;
                    }
                }
            }

            // empty q and reset handles
            while (!q_.empty())
            {
                q_.top()->data->handle = nullptr;
                q_.pop();
            }
            q_.clear();

            // Checking for solution or iterative improvement
            if (checkForSolution)
            {
                bool updatedSolution = false;
                for (auto &goalMotion : goalMotions_)
                {
                    if (opt_->isCostBetterThan(goalMotion->cost, bestCost_))
                    {
                        if (opt_->isFinite(bestCost_) == false)
                        {
                            OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %u iterations (%u "
                                        "vertices in the graph)",
                                        getName().c_str(), goalMotion->cost.value(), iterations_, nn_->size());
                        }
                        bestCost_ = goalMotion->cost;
                        updatedSolution = true;
                    }

                    sufficientlyShort = opt_->isSatisfied(goalMotion->cost);
                    if (sufficientlyShort)
                    {
                        solution = goalMotion;
                        break;
                    }
                    else if (!solution || opt_->isCostBetterThan(goalMotion->cost, solution->cost))
                    {
                        solution = goalMotion;
                        updatedSolution = true;
                    }
                }

                if (updatedSolution)
                {
                    if (intermediateSolutionCallback)
                    {
                        std::vector<const base::State *> spath;
                        Motion *intermediate_solution =
                            solution->parent;  // Do not include goal state to simplify code.

                        // Push back until we find the start, but not the start itself
                        while (intermediate_solution->parent != nullptr)
                        {
                            spath.push_back(intermediate_solution->state);
                            intermediate_solution = intermediate_solution->parent;
                        }

                        intermediateSolutionCallback(this, spath, bestCost_);
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

    bool approximate = (solution == nullptr);
    bool addedSolution = false;
    if (approximate)
        solution = approximation;
    else
        lastGoalMotion_ = solution;

    if (solution != nullptr)
    {
        ptc.terminate();
        // construct the solution path
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // set the solution path
        auto path = std::make_shared<PathGeometric>(si_);
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);

        // Add the solution path.
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());
        if (approximate)
            psol.setApproximate(approximatedist);
        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, bestCost_, sufficientlyShort);
        pdef_->addSolutionPath(psol);

        addedSolution = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree. Final solution cost "
                "%.3f",
                getName().c_str(), statesGenerated, rewireTest, goalMotions_.size(), bestCost_.value());

    return {addedSolution, approximate};
}

void ompl::geometric::RRTXstatic::updateQueue(Motion *x)
{
    // If x->handle is not NULL, x is already in the queue and needs to be update, otherwise it is inserted
    if (x->handle != nullptr)
    {
        q_.update(x->handle);
    }
    else
    {
        x->handle = q_.insert(x);
    }
}

void ompl::geometric::RRTXstatic::removeFromParent(Motion *m)
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

void ompl::geometric::RRTXstatic::calculateRRG()
{
    auto cardDbl = static_cast<double>(nn_->size() + 1u);
    rrg_k_ = std::ceil(k_rrt_ * log(cardDbl));
    rrg_r_ = std::min(maxDistance_,
                      r_rrt_ * std::pow(log(cardDbl) / cardDbl, 1 / static_cast<double>(si_->getStateDimension())));
}

void ompl::geometric::RRTXstatic::getNeighbors(Motion *motion) const
{
    if (motion->nbh.size() > 0)
    {
        return;
    }

    std::vector<Motion *> nbh;
    if (useKNearest_)
    {
        //- k-nearest RRT*
        nn_->nearestK(motion, rrg_k_, nbh);
    }
    else
    {
        nn_->nearestR(motion, rrg_r_, nbh);
    }

    motion->nbh.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), motion->nbh.begin(),
                   [](Motion *m) { return std::pair<Motion *, bool>(m, false); });
}

bool ompl::geometric::RRTXstatic::includeVertex(const Motion *x) const
{
    switch (variant_)
    {
        case 1:
            return opt_->isCostBetterThan(mc_.alphaCostPlusHeuristic(x, alpha_), opt_->infiniteCost());  // Always true?
        case 2:
            return opt_->isCostBetterThan(mc_.alphaCostPlusHeuristic(x->parent, alpha_), bestCost_);
        case 3:
            return opt_->isCostBetterThan(mc_.alphaCostPlusHeuristic(x, alpha_), bestCost_);
        default:  // no rejection
            return true;
    }
}

void ompl::geometric::RRTXstatic::freeMemory()
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

void ompl::geometric::RRTXstatic::getPlannerData(base::PlannerData &data) const
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

void ompl::geometric::RRTXstatic::setInformedSampling(bool informedSampling)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setSampleRejection, assert that:
    if (informedSampling == true && useRejectionSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // Check if we're changing the setting of informed sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (informedSampling != useInformedSampling_)
    {
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

void ompl::geometric::RRTXstatic::setSampleRejection(const bool reject)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setSampleRejection, assert that:
    if (reject == true && useInformedSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // Check if we're changing the setting of rejection sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (reject != useRejectionSampling_)
    {
        // Store the setting
        useRejectionSampling_ = reject;

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

void ompl::geometric::RRTXstatic::allocSampler()
{
    // Allocate the appropriate type of sampler.
    if (useInformedSampling_)
    {
        // We are using informed sampling, this can end-up reverting to rejection sampling in some cases
        OMPL_INFORM("%s: Using informed sampling.", getName().c_str());
        infSampler_ = opt_->allocInformedStateSampler(pdef_, numSampleAttempts_);
    }
    else if (useRejectionSampling_)
    {
        // We are explicitly using rejection sampling.
        OMPL_INFORM("%s: Using rejection sampling.", getName().c_str());
        infSampler_ = std::make_shared<base::RejectionInfSampler>(pdef_, numSampleAttempts_);
    }
    else
    {
        // We are using a regular sampler
        sampler_ = si_->allocStateSampler();
    }
}

bool ompl::geometric::RRTXstatic::sampleUniform(base::State *statePtr)
{
    // Use the appropriate sampler
    if (useInformedSampling_ || useRejectionSampling_)
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

void ompl::geometric::RRTXstatic::calculateRewiringLowerBounds()
{
    auto dimDbl = static_cast<double>(si_->getStateDimension());

    // k_rrt > 2^(d + 1) * e * (1 + 1 / d).  K-nearest RRT*
    k_rrt_ = rewireFactor_ * (std::pow(2, dimDbl + 1) * boost::math::constants::e<double>() * (1.0 + 1.0 / dimDbl));

    // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
    r_rrt_ = rewireFactor_ *
             std::pow(2 * (1.0 + 1.0 / dimDbl) * (si_->getSpaceMeasure() / unitNBallMeasure(si_->getStateDimension())),
                      1.0 / dimDbl);
}

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Oxford
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
 *   * Neither the name of the University of Toronto nor the names of its
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

// Authors: Marlin Strub

#include "ompl/geometric/planners/eitstar/RandomGeometricGraph.h"

#include <cmath>

#include <boost/math/constants/constants.hpp>

#include "ompl/base/OptimizationObjective.h"
#include "ompl/util/GeometricEquations.h"

#include "ompl/geometric/planners/eitstar/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        namespace eitstar
        {
            RandomGeometricGraph::RandomGeometricGraph(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo,
                                                       const ompl::base::Cost &solutionCost)

              : samples_(8 /* degree */, 4 /* min degree */, 12 /* max degree */, 50 /* max num points per leaf */,
                         500 /* removed cache size */,
                         false /* rebalancing */)  // These are the defaults. Play with them.
              , spaceInfo_(spaceInfo)
              , space_(spaceInfo->getStateSpace())
              , dimension_(spaceInfo->getStateDimension())
              , unitNBallMeasure_(unitNBallMeasure(spaceInfo->getStateDimension()))
              , solutionCost_(solutionCost)
            {
                samples_.setDistanceFunction(
                    [this](const std::shared_ptr<State> &state1, const std::shared_ptr<State> &state2) {
                        return spaceInfo_->distance(state1->state_, state2->state_);
                    });
            }

            void RandomGeometricGraph::setup(const std::shared_ptr<ompl::base::ProblemDefinition> &problem,
                                             ompl::base::PlannerInputStates *inputStates)
            {
                problem_ = problem;
                objective_ = problem->getOptimizationObjective();
                k_rgg_ = boost::math::constants::e<double>() +
                         (boost::math::constants::e<double>() / spaceInfo_->getStateDimension());
                updateStartAndGoalStates(ompl::base::plannerAlwaysTerminatingCondition(), inputStates);
            }

            void RandomGeometricGraph::clear()
            {
                problem_.reset();
                objective_.reset();
                sampler_.reset();
                k_rgg_ = std::numeric_limits<std::size_t>::max();
                numNeighbors_ = std::numeric_limits<std::size_t>::max();
                radius_ = std::numeric_limits<double>::infinity();
                samples_.clear();
                newSamples_.clear();
                startStates_.clear();
                goalStates_.clear();
                prunedStartStates_.clear();
                prunedGoalStates_.clear();
            }

            void RandomGeometricGraph::clearQuery()
            {
                startStates_.clear();
                goalStates_.clear();
                prunedStartStates_.clear();
                prunedGoalStates_.clear();
            }

            void RandomGeometricGraph::updateStartAndGoalStates(
                const ompl::base::PlannerTerminationCondition &terminationCondition,
                ompl::base::PlannerInputStates *inputStates)
            {
                // We need to keep track of whether a new goal and/or a new start has been added.
                bool addedNewStartState = false;
                bool addedNewGoalState = false;

                // First update the goals. We have to call inputStates->nextGoal(terminationCondition) at least once
                // (regardless of the return value of inputStates->moreGoalStates()) in case the termination condition
                // wants us to wait for a goal.
                if (goalStates_.size() < maxNumGoals_)
                {
                    do
                    {
                        // Get a new goal. If there are none, or the underlying state is invalid this will be a nullptr.
                        const auto newGoalState =
                            inputStates->nextGoal(ompl::base::plannerAlwaysTerminatingCondition());

                        // If there was a new valid goal, register it as such and remember that a goal has been added.
                        if (static_cast<bool>(newGoalState))
                        {
                            registerGoalState(newGoalState);
                            addedNewGoalState = true;
                        }
                    } while (inputStates->haveMoreGoalStates() && goalStates_.size() < maxNumGoals_ &&
                             !terminationCondition);
                }

                // Having updated the goals, we now update the starts.
                while (inputStates->haveMoreStartStates())
                {
                    // Get the next start. The returned pointer can be a nullptr (if the state is invalid).
                    const auto newStartState = inputStates->nextStart();

                    // If there is a new valid start, register it as such and remember that a start has been added.
                    if (static_cast<bool>(newStartState))
                    {
                        registerStartState(newStartState);
                        addedNewStartState = true;
                    }
                }

                // If we added a new start and have previously pruned goals, we might want to add the goals back.
                if (addedNewStartState && !prunedGoalStates_.empty())
                {
                    // Keep track of the pruned goal vertices that have been revived.
                    std::vector<std::vector<std::shared_ptr<State>>::iterator> revivedGoals;

                    // Let's see if the pruned goal is close enough to any new start to revive it..
                    for (auto it = prunedGoalStates_.begin(); it != prunedGoalStates_.end(); ++it)
                    {
                        // Loop over all start states to get the best cost.
                        auto heuristicCost = objective_->infiniteCost();
                        for (const auto &start : startStates_)
                        {
                            heuristicCost = objective_->betterCost(
                                heuristicCost, objective_->motionCostHeuristic(start->raw(), (*it)->raw()));
                        }

                        // If this goal can possibly improve the current solution, add it back to the graph.
                        if (objective_->isCostBetterThan(heuristicCost, solutionCost_))
                        {
                            registerGoalState((*it)->raw());
                            addedNewGoalState = true;
                            revivedGoals.emplace_back(it);
                        }
                    }

                    // Remove all revived goals from the pruned goals.
                    for (auto &revivedGoal : revivedGoals)
                    {
                        std::iter_swap(revivedGoal, prunedGoalStates_.rbegin());
                        prunedGoalStates_.pop_back();
                    }
                }

                // If we added a new goal and have previously pruned starts, we might want to add the starts back.
                if (addedNewGoalState && !prunedStartStates_.empty())
                {
                    // Keep track of the pruned goal vertices that have been revived.
                    std::vector<std::vector<std::shared_ptr<State>>::iterator> revivedStarts;

                    // Let's see if the pruned start is close enough to any new goal to revive it..
                    for (auto it = prunedStartStates_.begin(); it != prunedStartStates_.end(); ++it)
                    {
                        // Loop over all start states to get the best cost.
                        auto heuristicCost = objective_->infiniteCost();
                        for (const auto &goal : goalStates_)
                        {
                            heuristicCost = objective_->betterCost(
                                heuristicCost, objective_->motionCostHeuristic(goal->raw(), (*it)->raw()));
                        }

                        // If this start can possibly improve the current solution, add it back to the graph.
                        if (objective_->isCostBetterThan(heuristicCost, solutionCost_))
                        {
                            registerStartState((*it)->raw());
                            addedNewStartState = true;
                            revivedStarts.emplace_back(it);
                        }
                    }

                    // Remove all revived starts from the pruned starts.
                    for (auto &revivedStart : revivedStarts)
                    {
                        std::iter_swap(revivedStart, prunedStartStates_.rbegin());
                        prunedStartStates_.pop_back();
                    }
                }

                if (addedNewGoalState || addedNewStartState)
                {
                    // Allocate a state sampler if we have at least one start and one goal.
                    if (!startStates_.empty() && !goalStates_.empty())
                    {
                        sampler_ =
                            objective_->allocInformedStateSampler(problem_, std::numeric_limits<unsigned int>::max());
                    }
                }

                if (!goalStates_.empty() && startStates_.empty())
                {
                    OMPL_WARN("EIT*: The problem has a goal but not a start. EIT* can not find a solution since "
                              "PlannerInputStates provides no method to wait for a valid start state to appear.");
                }

                // Compute the minimum possible cost for this problem given the start and goal states and an admissible
                // cost heuristic.
                minPossibleCost_ = objective_->infiniteCost();
                for (const auto &start : startStates_)
                {
                    for (const auto &goal : goalStates_)
                    {
                        minPossibleCost_ = objective_->betterCost(
                            minPossibleCost_, objective_->motionCostHeuristic(start->raw(), goal->raw()));
                    }
                }
            }

            ompl::base::Cost RandomGeometricGraph::minPossibleCost() const
            {
                return minPossibleCost_;
            }

            void RandomGeometricGraph::setRadiusFactor(double factor)
            {
                radiusFactor_ = factor;
            }

            double RandomGeometricGraph::getRadiusFactor() const
            {
                return radiusFactor_;
            }

            const std::vector<std::shared_ptr<State>> &RandomGeometricGraph::getStartStates() const
            {
                return startStates_;
            }

            const std::vector<std::shared_ptr<State>> &RandomGeometricGraph::getGoalStates() const
            {
                return goalStates_;
            }

            unsigned int RandomGeometricGraph::getNumberOfSampledStates() const
            {
                return numSampledStates_;
            }

            unsigned int RandomGeometricGraph::getNumberOfValidSamples() const
            {
                return numValidSamples_;
            }

            unsigned int RandomGeometricGraph::getNumberOfNearestNeighborCalls() const
            {
                return numNearestNeighborCalls_;
            }

            bool RandomGeometricGraph::hasStartState() const
            {
                return !startStates_.empty();
            }

            bool RandomGeometricGraph::hasGoalState() const
            {
                return !goalStates_.empty();
            }

            bool RandomGeometricGraph::isStart(const std::shared_ptr<State> &state) const
            {
                return std::any_of(startStates_.begin(), startStates_.end(),
                                   [&state](const auto &start) { return state->getId() == start->getId(); });
            }

            bool RandomGeometricGraph::isGoal(const std::shared_ptr<State> &state) const
            {
                return std::any_of(goalStates_.begin(), goalStates_.end(),
                                   [&state](const auto &goal) { return state->getId() == goal->getId(); });
            }

            std::vector<std::shared_ptr<State>> RandomGeometricGraph::getStates() const
            {
                std::vector<std::shared_ptr<State>> samples;
                samples_.list(samples);
                return samples;
            }

            void RandomGeometricGraph::registerInvalidEdge(const Edge &edge) const
            {
                // Remove the edge from the caches (using the erase-remove idiom).
                auto &sourceNeighbors = edge.source->neighbors_.second;
                auto &targetNeighbors = edge.target->neighbors_.second;
                sourceNeighbors.erase(
                    std::remove_if(sourceNeighbors.begin(), sourceNeighbors.end(),
                                   [&edge](const auto &neighbor) { return neighbor->getId() == edge.target->getId(); }),
                    sourceNeighbors.end());
                targetNeighbors.erase(
                    std::remove_if(targetNeighbors.begin(), targetNeighbors.end(),
                                   [&edge](const auto &neighbor) { return neighbor->getId() == edge.source->getId(); }),
                    targetNeighbors.end());
            }

            std::size_t RandomGeometricGraph::getTag() const
            {
                return tag_;
            }

            std::shared_ptr<State> RandomGeometricGraph::registerStartState(const ompl::base::State *state)
            {
                // Allocate the start state.
                auto start = std::make_shared<State>(spaceInfo_, objective_);
                spaceInfo_->copyState(start->raw(), state);

                // Hold onto it.
                startStates_.emplace_back(start);
                samples_.add(start);

                // Initialize the state.
                initializeState(start);

                // Ensure its lower bounds are correct.
                assert(objective_->isCostEquivalentTo(start->getLowerBoundCostToCome(), objective_->identityCost()));
                assert(start->getLowerBoundEffortToCome() == 0u);

                return start;
            }

            std::shared_ptr<State> RandomGeometricGraph::registerGoalState(const ompl::base::State *state)
            {
                // Allocate the goal state.
                auto goal = std::make_shared<State>(spaceInfo_, objective_);
                spaceInfo_->copyState(goal->raw(), state);

                // Hold onto it.
                goalStates_.emplace_back(goal);
                samples_.add(goal);

                // Initialize the state.
                initializeState(goal);

                // Ensure its lower bounds are correct.
                assert(objective_->isCostEquivalentTo(goal->getLowerBoundCostToGo(), objective_->identityCost()));

                return goal;
            }

            bool RandomGeometricGraph::addStates(std::size_t numNewStates,
                                                 const ompl::base::PlannerTerminationCondition &terminationCondition)
            {
                // Assert sanity of used variables.
                assert(sampler_);
                assert(objective_);

                if (numNewStates == 0u)
                {
                    return true;
                }

                // Create the requested number of new states.
                do
                {
                    // Allocate a new state.
                    auto state = std::make_shared<State>(spaceInfo_, objective_);
                    newSamples_.emplace_back(state);

                    do  // Sample randomly until a valid state is found.
                    {
                        sampler_->sampleUniform(state->raw(), solutionCost_);
                        ++numSampledStates_;
                    } while (!spaceInfo_->isValid(state->raw()));

                    // Add this state to the goal states if it is a goal.
                    if (problem_->getGoal()->isSatisfied(state->raw()))
                    {
                        goalStates_.emplace_back(state);
                    }

                    // Initialize the state.
                    initializeState(state);

                    // We've found a valid sample.
                    ++numValidSamples_;
                } while (newSamples_.size() < numNewStates && !terminationCondition);

                // Add the new states to the samples.
                if (newSamples_.size() == numNewStates)
                {
                    // Count the number of informed states before adding the new states. This saves some counting,
                    // because all new states will be in the informed set, and its known how many of them will be added.
                    // If pruning is enabled, we can do this now. After pruning all remaining states are in the informed
                    // set.
                    auto numInformedSamples{0u};
                    if (isPruningEnabled_)
                    {
                        prune();
                        numInformedSamples = samples_.size();
                    }
                    else
                    {
                        numInformedSamples = countSamplesInInformedSet();
                    }

                    samples_.add(newSamples_);
                    newSamples_.clear();

                    // Update the radius by considering all informed states.
                    if (useKNearest_)
                    {
                        numNeighbors_ = computeNumberOfNeighbors(numInformedSamples + numNewStates);
                    }
                    else
                    {
                        radius_ = computeRadius(numInformedSamples + numNewStates);
                    }

                    // Update the tag.
                    ++tag_;

                    return true;
                }
                return false;
            }

            void RandomGeometricGraph::enablePruning(bool prune)
            {
                isPruningEnabled_ = prune;
            }

            bool RandomGeometricGraph::isPruningEnabled() const
            {
                return isPruningEnabled_;
            }

            void RandomGeometricGraph::setUseKNearest(bool useKNearest)
            {
                useKNearest_ = useKNearest;
            }

            bool RandomGeometricGraph::getUseKNearest() const
            {
                return useKNearest_;
            }

            void RandomGeometricGraph::setMaxNumberOfGoals(unsigned int maxNumberOfGoals)
            {
                maxNumGoals_ = maxNumberOfGoals;
            }

            unsigned int RandomGeometricGraph::getMaxNumberOfGoals() const
            {
                return maxNumGoals_;
            }

            std::vector<std::shared_ptr<State>>
            RandomGeometricGraph::getNeighbors(const std::shared_ptr<State> &state) const
            {
                assert(state);
                // If the neighbors cache of the vertex isn't up to date, update it.
                if (state->neighbors_.first != tag_)
                {
                    // The cache is invalid, let's clear all vertices.
                    state->neighbors_.second.clear();

                    // Get the neighbors by performing a nearest neighbor search.
                    std::vector<std::shared_ptr<State>> neighbors;
                    if (useKNearest_)
                    {
                        samples_.nearestK(state, numNeighbors_, neighbors);
                    }
                    else
                    {
                        samples_.nearestR(state, radius_, neighbors);
                    }
                    // We dont want to connect to blacklisted neighbors and the querying state itself.
                    const auto connectionPredicate = [&state, this](const std::shared_ptr<State> &neighbor) {
                        return (state->blacklist_.find(neighbor->id_) == state->blacklist_.end()) &&
                               (state->id_ != neighbor->id_) && !(isGoal(state) && isGoal(neighbor));
                    };

                    // Cache the neighbors that are not blacklisted and not the state itself.
                    std::copy_if(neighbors.begin(), neighbors.end(), std::back_inserter(state->neighbors_.second),
                                 connectionPredicate);

                    // Update the tag of the cache.
                    state->neighbors_.first = tag_;

                    // Increase the counter of nearest neighbor calls.
                    ++numNearestNeighborCalls_;
                }

                // The cache is now guaranteed to be up to date.
                auto neighbors = state->neighbors_.second;

                // Add the forward parent and children.
                if (state->hasForwardVertex())
                {
                    const auto forwardVertex = state->asForwardVertex();

                    // Add the parent.
                    if (auto forwardParent = forwardVertex->getParent().lock())
                    {
                        neighbors.emplace_back(forwardParent->getState());
                    }

                    // Add the children.
                    const auto &forwardChildren = forwardVertex->getChildren();
                    for (const auto &child : forwardChildren)
                    {
                        neighbors.emplace_back(child->getState());
                    }
                }

                // Add the reverse parent and children.
                if (state->hasReverseVertex())
                {
                    const auto reverseVertex = state->asReverseVertex();

                    // Add the parent.
                    if (auto reverseParent = reverseVertex->getParent().lock())
                    {
                        neighbors.emplace_back(reverseParent->getState());
                    }

                    // Add the children.
                    const auto &reverseChildren = reverseVertex->getChildren();
                    for (const auto &child : reverseChildren)
                    {
                        neighbors.emplace_back(child->getState());
                    }
                }

                return neighbors;
            }

            void RandomGeometricGraph::prune()
            {
                // Is there really no way of doing this without looping over all samples?
                std::vector<std::shared_ptr<State>> samples;
                samples_.list(samples);

                // Prepare a vector of samples to be pruned.
                std::vector<std::shared_ptr<State>> samplesToBePruned;

                // Check each sample if it can be pruned.
                for (const auto &sample : samples)
                {
                    if (canBePruned(sample))
                    {
                        if (isStart(sample))
                        {
                            prunedStartStates_.emplace_back(sample);
                        }
                        else if (isGoal(sample))
                        {
                            prunedGoalStates_.emplace_back(sample);
                        }

                        samplesToBePruned.emplace_back(sample);
                    }
                }

                // Remove all samples to be pruned.
                for (const auto &sample : samplesToBePruned)
                {
                    // Remove the sample from the graph.
                    samples_.remove(sample);

                    // Remove it from both search trees.
                    if (sample->hasForwardVertex())
                    {
                        auto forwardVertex = sample->asForwardVertex();
                        if (auto parent = forwardVertex->getParent().lock())
                        {
                            forwardVertex->resetParent();
                            parent->removeChild(forwardVertex);
                        }
                    }
                    if (sample->hasReverseVertex())
                    {
                        auto reverseVertex = sample->asReverseVertex();
                        if (auto parent = reverseVertex->getParent().lock())
                        {
                            reverseVertex->resetParent();
                            parent->removeChild(reverseVertex);
                        }
                    }
                }

                // If any sample was pruned, this has invalidated the nearest neighbor cache.
                if (!samplesToBePruned.empty())
                {
                    ++tag_;
                }
            }

            std::size_t RandomGeometricGraph::countSamplesInInformedSet() const
            {
                // Is there really no way of doing this without looping over all samples?
                std::vector<std::shared_ptr<State>> samples;
                samples_.list(samples);

                // Count the number of samples that can possibly improve the solution and subtract the start and goal
                // states from this number, as they're not technically uniformly distributed.
                return std::count_if(samples.begin(), samples.end(),
                                     [this](const auto &sample) { return !canBePruned(sample); }) -
                       startStates_.size() - goalStates_.size();
            }

            bool RandomGeometricGraph::canBePruned(const std::shared_ptr<State> &state) const
            {
                // If we don't have a solution, no state can be pruned.
                if (!objective_->isFinite(solutionCost_))
                {
                    return false;
                }
                else
                {
                    // Get the heuristic cost to come.
                    const auto costToCome = lowerBoundCostToCome(state);

                    // Get the heuristic cost to go.
                    const auto costToGo = lowerBoundCostToGo(state);

                    // Return whether the current solution is better than the lower bound potential solution.
                    return objective_->isCostBetterThan(solutionCost_, objective_->combineCosts(costToCome, costToGo));
                }
            }

            ompl::base::Cost RandomGeometricGraph::lowerBoundCostToCome(const std::shared_ptr<State> &state) const
            {
                // Get the preferred start for this state.
                auto bestCost = objective_->infiniteCost();
                for (const auto &start : startStates_)
                {
                    bestCost =
                        objective_->betterCost(objective_->motionCostHeuristic(start->raw(), state->raw()), bestCost);
                }

                return bestCost;
            }

            unsigned int RandomGeometricGraph::lowerBoundEffortToCome(const std::shared_ptr<State> &state) const
            {
                auto admissibleEffort = std::numeric_limits<unsigned int>::max();
                for (const auto &start : startStates_)
                {
                    admissibleEffort =
                        std::min(admissibleEffort, space_->validSegmentCount(start->raw(), state->raw()));
                }
                return admissibleEffort;
            }

            ompl::base::Cost RandomGeometricGraph::lowerBoundCostToGo(const std::shared_ptr<State> &state) const
            {
                // Get the preferred goal for this state.
                auto bestCost = objective_->infiniteCost();
                for (const auto &goal : goalStates_)
                {
                    bestCost =
                        objective_->betterCost(objective_->motionCostHeuristic(state->raw(), goal->raw()), bestCost);
                }

                return bestCost;
            }

            void RandomGeometricGraph::initializeState(const std::shared_ptr<State> &state)
            {
                // Set the current cost to come.
                state->setCurrentCostToCome(objective_->infiniteCost());

                // Set the lower bounds.
                state->setLowerBoundCostToCome(lowerBoundCostToCome(state));
                state->setLowerBoundEffortToCome(lowerBoundEffortToCome(state));
                state->setLowerBoundCostToGo(lowerBoundCostToGo(state));

                if (problem_->getGoal()->isSatisfied(state->raw()))
                {
                    // Set the cost to go estimate admissible for the approximation.
                    state->setAdmissibleCostToGo(objective_->identityCost());

                    // Set the estimated cost to go.
                    state->setEstimatedCostToGo(objective_->identityCost());

                    // Set the estimated effort to go.
                    state->setEstimatedEffortToGo(0u);
                }
                else
                {
                    // Set the admissible cost to go.
                    state->setAdmissibleCostToGo(objective_->infiniteCost());

                    // Set the estimated cost to go.
                    state->setEstimatedCostToGo(objective_->infiniteCost());

                    // Set the estimated effort to go.
                    state->setEstimatedEffortToGo(std::numeric_limits<std::size_t>::max());
                }

                if (isStart(state)) {
                    state->setCurrentCostToCome(objective_->identityCost());
                }
            }

            std::size_t RandomGeometricGraph::computeNumberOfNeighbors(std::size_t numInformedSamples) const
            {
                return std::ceil(radiusFactor_ * k_rgg_ * std::log(static_cast<double>(numInformedSamples)));
            }

            double RandomGeometricGraph::computeRadius(std::size_t numInformedSamples) const
            {
                // Compute and return the radius. Note to self: double / int -> double. You looked it up. It's fine.
                // RRT*
                return radiusFactor_ *
                       std::pow(2.0 * (1.0 + 1.0 / dimension_) *
                                    (sampler_->getInformedMeasure(solutionCost_) / unitNBallMeasure_) *
                                    (std::log(static_cast<double>(numInformedSamples)) / numInformedSamples),
                                1.0 / dimension_);

                // FMT*
                // return 2.0 * radiusFactor_ *
                //        std::pow((1.0 / dimension_) * (sampler_->getInformedMeasure(solutionCost_) /
                //        unitNBallMeasure_) *
                //                     (std::log(static_cast<double>(numInformedSamples)) / numInformedSamples),
                //                 1.0 / dimension_);
            }

        }  // namespace eitstar

    }  // namespace geometric

}  // namespace ompl
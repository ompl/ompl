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

#include "ompl/geometric/planners/eitstar/EITstar.h"

#include <algorithm>
#include <memory>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/planners/eitstar/stopwatch/timetable.h"

using namespace std::string_literals;
using namespace ompl::geometric::eitstar;

namespace ompl
{
    namespace geometric
    {
        EITstar::EITstar(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo)
          : ompl::base::Planner(spaceInfo, "EIT*")
          , graph_(spaceInfo, solutionCost_)
          , detectionState_(spaceInfo->allocState())
          , space_(spaceInfo->getStateSpace())
          , motionValidator_(spaceInfo->getMotionValidator())
          , solutionCost_()
        {
            // Specify EIT*'s planner specs.
            specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
            specs_.multithreaded = false;
            specs_.approximateSolutions = true;
            specs_.optimizingPaths = true;
            specs_.directed = true;
            specs_.provingSolutionNonExistence = false;
            specs_.canReportIntermediateSolutions = true;

            // Register the setting callbacks.
            declareParam<bool>("use_k_nearest", this, &EITstar::setUseKNearest, &EITstar::getUseKNearest, "0,1");
            declareParam<double>("rewire_factor", this, &EITstar::setRadiusFactor, &EITstar::getRadiusFactor,
                                 "1.0:0.01:3.0");
            declareParam<std::size_t>("batch_size", this, &EITstar::setBatchSize, &EITstar::getBatchSize, "1:1:10000");
            declareParam<bool>("use_graph_pruning", this, &EITstar::enablePruning, &EITstar::isPruningEnabled, "0,1");
            declareParam<bool>("find_approximate_solutions", this, &EITstar::trackApproximateSolutions,
                               &EITstar::areApproximateSolutionsTracked, "0,1");
            declareParam<unsigned int>("set_max_num_goals", this, &EITstar::setMaxNumberOfGoals,
                                       &EITstar::getMaxNumberOfGoals, "1:1:1000");

            // Register the progress properties.
            addPlannerProgressProperty("iterations INTEGER", [this]() { return std::to_string(iteration_); });
            addPlannerProgressProperty("best cost DOUBLE", [this]() { return std::to_string(solutionCost_.value()); });
            addPlannerProgressProperty("state collision checks INTEGER",
                                       [this]() { return std::to_string(graph_.getNumberOfSampledStates()); });
            addPlannerProgressProperty("edge collision checks INTEGER",
                                       [this]() { return std::to_string(numCollisionCheckedEdges_); });
            addPlannerProgressProperty("nearest neighbour calls INTEGER",
                                       [this]() { return std::to_string(graph_.getNumberOfNearestNeighborCalls()); });
        }

        EITstar::~EITstar()
        {
            spaceInfo_->freeState(detectionState_);
        }

        void EITstar::setup()
        {
            // Call the base class setup.
            Planner::setup();

            // Check that the problem definition is set.
            if (static_cast<bool>(problem_))
            {
                // If we were given a goal, make sure its of appropriate type.
                if (!(problem_->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION)))
                {
                    OMPL_ERROR("EIT* is currently only implemented for goals that can be cast to "
                               "ompl::base::GOAL_SAMPLEABLE_REGION.");
                    setup_ = false;
                    return;
                }

                // Default to path length optimization if no objective has been specified.
                if (!problem_->hasOptimizationObjective())
                {
                    OMPL_WARN("%s: No optimization objective has been specified. The default is optimizing path "
                              "length.",
                              name_.c_str());
                    problem_->setOptimizationObjective(
                        std::make_shared<ompl::base::PathLengthOptimizationObjective>(spaceInfo_));
                }

                // Pull through the optimization objective for direct access.
                objective_ = problem_->getOptimizationObjective();

                // Initialize costs to infinity.
                solutionCost_ = objective_->infiniteCost();
                reverseCost_ = objective_->infiniteCost();

                // Instantiate the queues.
                forwardQueue_ = std::make_unique<eitstar::ForwardQueue>(objective_, space_);
                reverseQueue_ = std::make_unique<eitstar::ReverseQueue>(objective_, space_);

                // Setup the graph with the problem information.
                graph_.setup(problem_, &pis_);

                // Create the start vertices.
                for (const auto &start : graph_.getStartStates())
                {
                    startVertices_.emplace_back(start->asForwardVertex());
                }

                // Create the goal vertices.
                for (const auto &goal : graph_.getGoalStates())
                {
                    goalVertices_.emplace_back(goal->asReverseVertex());
                }

                // Populate the queues.
                expandGoalVerticesIntoReverseQueue();
                expandStartVerticesIntoForwardQueue();
            }
            else
            {
                setup_ = false;
                OMPL_WARN("%s: Unable to setup without a problem definition.", name_.c_str());
            }
        }

        ompl::base::PlannerStatus EITstar::solve(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // Check that the planner and state space are setup.
            auto status = ensureSetup();

            // Return early if the planner or state space are not setup.
            if (status == ompl::base::PlannerStatus::StatusType::ABORT)
            {
                return status;
            }

            // Update the status of the planner.
            status = ensureStartAndGoalStates(terminationCondition);

            // Return early if no problem can be solved.
            if (status == ompl::base::PlannerStatus::StatusType::INVALID_START ||
                status == ompl::base::PlannerStatus::StatusType::INVALID_GOAL)
            {
                return status;
            }

            // Let the world know what we're doing.
            OMPL_INFORM("%s: Solving the given planning problem. The current best solution cost is %.4f", name_.c_str(),
                        solutionCost_.value());

            // Iterate until stopped or objective is satisfied.
            while (!terminationCondition && !objective_->isSatisfied(solutionCost_))
            {
                iterate(terminationCondition);
            }

            // Someone might call ProblemDefinition::clearSolutionPaths() between invocations of Planner::sovle(), in
            // which case previously found solutions are not registered with the problem definition anymore.
            status = updateSolution();

            // Let the caller know the status.
            informAboutPlannerStatus(status);
            return status;
        }

        ompl::base::Cost EITstar::bestCost() const
        {
            return solutionCost_;
        }

        void EITstar::setBatchSize(unsigned int numSamples)
        {
            batchSize_ = numSamples;
        }

        unsigned int EITstar::getBatchSize() const
        {
            return batchSize_;
        }

        void EITstar::setInitialNumberOfSparseCollisionChecks(std::size_t numChecks)
        {
            initialNumSparseCollisionChecks_ = numChecks;
            numSparseCollisionChecksCurrentLevel_ = numChecks;
            numSparseCollisionChecksPreviousLevel_ = 0u;
        }

        void EITstar::setRadiusFactor(double factor)
        {
            graph_.setRadiusFactor(factor);
        }

        double EITstar::getRadiusFactor() const
        {
            return graph_.getRadiusFactor();
        }

        void EITstar::setSuboptimalityFactor(double factor)
        {
            suboptimalityFactor_ = factor;
        }

        void EITstar::enablePruning(bool enable)
        {
            graph_.enablePruning(enable);
        }

        bool EITstar::isPruningEnabled() const
        {
            return graph_.isPruningEnabled();
        }

        void EITstar::trackApproximateSolutions(bool track)
        {
            trackApproximateSolutions_ = track;
        }

        bool EITstar::areApproximateSolutionsTracked() const
        {
            return trackApproximateSolutions_;
        }

        void EITstar::setUseKNearest(bool useKNearest)
        {
            graph_.setUseKNearest(useKNearest);
        }

        bool EITstar::getUseKNearest() const
        {
            return graph_.getUseKNearest();
        }

        void EITstar::setMaxNumberOfGoals(unsigned int numberOfGoals)
        {
            graph_.setMaxNumberOfGoals(numberOfGoals);
        }

        unsigned int EITstar::getMaxNumberOfGoals() const
        {
            return graph_.getMaxNumberOfGoals();
        }

        std::vector<Edge> EITstar::getForwardQueue() const
        {
            return forwardQueue_->getEdges();
        }

        std::vector<Edge> EITstar::getReverseQueue() const
        {
            return reverseQueue_->getEdges();
        }

        std::vector<Edge> EITstar::getReverseTree() const
        {
            // Prepare the return value.
            std::vector<Edge> edges;

            // Define a helper that recursively gets all reverse edges of a vertex.
            std::function<void(const std::shared_ptr<Vertex> &)> getEdgesRecursively =
                [&edges, &getEdgesRecursively](const std::shared_ptr<Vertex> &vertex) {
                    for (const auto &child : vertex->getChildren())
                    {
                        getEdgesRecursively(child);
                    }
                    // Catch the root case.
                    if (auto parent = vertex->getParent().lock())
                    {
                        edges.emplace_back(parent->getState(), vertex->getState());
                    }
                };

            // Get the edges of all reverse roots recursively.
            for (const auto &root : goalVertices_)
            {
                getEdgesRecursively(root);
            }

            // Return all edges in the reverse tree.
            return edges;
        }

        Edge EITstar::getNextForwardEdge() const
        {
            assert(forwardQueue_);
            if (forwardQueue_->empty())
            {
                return {};
            }
            return forwardQueue_->peek(suboptimalityFactor_);
        }

        Edge EITstar::getNextReverseEdge() const
        {
            assert(reverseQueue_);
            if (reverseQueue_->empty())
            {
                return {};
            }
            return reverseQueue_->peek();
        }

        void EITstar::getPlannerData(base::PlannerData &data) const
        {
            // base::PlannerDataVertex takes a raw pointer to a state. I want to guarantee, that the state lives as long
            // as the program lives.
            static std::set<std::shared_ptr<State>,
                            std::function<bool(const std::shared_ptr<State> &, const std::shared_ptr<State> &)>>
                liveStates([](const auto &lhs, const auto &rhs) { return lhs->getId() < rhs->getId(); });

            // Get the base class data.
            Planner::getPlannerData(data);

            // Add the samples and their outgoing edges.
            for (const auto &state : graph_.getStates())
            {
                // Add the state to the live states.
                liveStates.insert(state);

                // Add the state as a vertex.
                data.addVertex(base::PlannerDataVertex(state->raw(), state->getId()));

                // If the sample is in the forward tree, add the outgoing edges.
                if (state->hasForwardVertex())
                {
                    for (const auto &child : state->asForwardVertex()->getChildren())
                    {
                        data.addEdge(base::PlannerDataVertex(state->asForwardVertex()->getState()->raw(),
                                                             state->asForwardVertex()->getState()->getId()),
                                     base::PlannerDataVertex(child->getState()->raw(), child->getState()->getId()));
                    }
                }
            }
        }

        void EITstar::iterate(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // First check if the reverse search needs to be continued.
            if (continueReverseSearch())
            {
                iterateReverseSearch();
            }  // If the reverse search is suspended, check if the forward search needs to be continued.
            else if (continueForwardSearch())
            {
                iterateForwardSearch();
            }  // If neither the reverse nor the forward search needs to be continued, improve the approximation.
            else
            {
                improveApproximation(terminationCondition);
            }

            // Increment the iteration count.
            ++iteration_;
        }

        void EITstar::iterateForwardSearch()
        {
            // Ensure the forward queue is not empty.
            assert(!forwardQueue_->empty());

            // Get the top edge from the queue.
            auto edge = forwardQueue_->pop(suboptimalityFactor_);
            ++numProcessedEdges_;

            // Assert the validity of the edge.
            assert(edge.source->hasForwardVertex());
            assert(!std::isfinite(suboptimalityFactor_) || isClosed(edge.target->asReverseVertex()));

            // The edge is a freeby if its parent is already the parent of the child.
            if (isInForwardTree(edge))
            {
                // We expand the target into the queue unless the target is a goal.
                forwardQueue_->insertOrUpdate(expandUnlessGoal(edge.target));
                return;
            }

            // If the edge can not improve the forward tree, then we're done with it.
            if (!couldImproveForwardTree(edge))
            {
                return;
            }

            // The edge could possibly improve the tree, check if it is valid.
            if (isValid(edge))
            {
                // Compute the true edge cost and the target cost through this edge.
                const auto edgeCost = objective_->motionCost(edge.source->raw(), edge.target->raw());
                const auto targetCost = combine(edge.source->getCurrentCostToCome(), edgeCost);

                // Check if the edge can actually improve the forward path and tree.
                if (isBetter(targetCost, edge.target->getCurrentCostToCome()) &&
                    isBetter(combine(targetCost, edge.target->getAdmissibleCostToGo()), solutionCost_))
                {
                    // Convenience access to parent and child vertices.
                    auto source = edge.source->asForwardVertex();
                    auto target = edge.target->asForwardVertex();

                    // Update the parent of the child in the forward tree.
                    target->updateParent(source);

                    // Add the child to the parents children.
                    source->addChild(target);

                    // Set the edge cost associated with this parent.
                    target->setEdgeCost(edgeCost);

                    // Update the cost-to-come.
                    edge.target->setCurrentCostToCome(targetCost);

                    // Update the cost of the children.
                    const auto changedVertices = target->updateCurrentCostOfChildren(objective_);

                    // Reflect changes in queue and solution cost.
                    for (const auto &vertex : changedVertices)
                    {
                        // Update any edge in the queue.
                        forwardQueue_->updateIfExists({vertex->getParent().lock()->getState(), vertex->getState()});

                        // Update the solution if the vertex is a goal.
                        if (graph_.isGoal(vertex->getState()))
                        {
                            updateExactSolution(vertex->getState());
                        }
                    }

                    // Expand the outgoing edges into the queue unless this state is the goal state.
                    if (!graph_.isGoal(edge.target))
                    {
                        forwardQueue_->insertOrUpdate(expand(edge.target));
                    }
                    else  // It is the goal state, update the solution.
                    {
                        updateExactSolution(edge.target);
                    }
                }
            }
            else
            {
                // Check if the edge is used in the reverse tree.
                const bool inReverseTree = edge.source->asReverseVertex()->isParent(edge.target->asReverseVertex()) ||
                                           edge.target->asReverseVertex()->isParent(edge.source->asReverseVertex());

                // If this edge was in the reverse search tree, it could be updated.
                if (inReverseTree)
                {
                    // Remember the current number of collision checks and increase it.
                    numSparseCollisionChecksPreviousLevel_ = numSparseCollisionChecksCurrentLevel_;
                    numSparseCollisionChecksCurrentLevel_ = (2u * numSparseCollisionChecksPreviousLevel_) + 1u;

                    // Restart the reverse search.
                    restartReverseSearch();

                    // Rebuild the forward queue.
                    forwardQueue_->rebuild();
                }
            }
        }

        void EITstar::iterateReverseSearch()
        {
            // Ensure the reverse queue is not empty.
            assert(!reverseQueue_->empty());

            // Get the top edge from the queue.
            auto edge = reverseQueue_->pop();
            auto &source = edge.source;
            auto &target = edge.target;

            // The parent vertex must have an associated vertex in the tree.
            assert(source->hasReverseVertex());

            // Register the expansion of its parent.
            source->asReverseVertex()->registerExpansionInReverseSearch(reverseSearchTag_);

            // The edge is a freeby if its parent is already the parent of the child.
            if (isInReverseTree(edge))
            {
                // Simply expand the target into the queue.
                reverseQueue_->insertOrUpdate(expand(target));
                return;
            }

            // Check whether the edge could be valid.
            if (couldBeValid(edge))
            {
                // Compute the heuristic cost.
                const auto edgeCost = objective_->motionCostHeuristic(source->raw(), target->raw());

                // Incorporate the edge in the reverse tree if it provides an improvement.
                if (doesImproveReverseTree(edge, edgeCost))
                {
                    // Get the parent and child vertices.
                    auto parentVertex = source->asReverseVertex();
                    auto childVertex = target->asReverseVertex();

                    // The child must not be closed.
                    assert(!isClosed(childVertex));

                    // Update the parent of the child in the reverse tree.
                    childVertex->updateParent(parentVertex);

                    // Add the child to the children of the parent.
                    parentVertex->addChild(childVertex);

                    // Update the admissible cost to go.
                    target->setAdmissibleCostToGo(combine(source->getAdmissibleCostToGo(), edgeCost));

                    // Update the best cost estimate of the target state if this edge can improve it.
                    target->setEstimatedCostToGo(
                        objective_->betterCost(estimateCostToTarget(edge), target->getEstimatedCostToGo()));

                    // Update the best effort estimate of the target state if this edge can improve it.
                    target->setEstimatedEffortToGo(
                        std::min(estimateEffortToTarget(edge), target->getEstimatedEffortToGo()));

                    // If this edge improves the reverse cost, update it.
                    if (graph_.isStart(target) && isBetter(target->getAdmissibleCostToGo(), reverseCost_))
                    {
                        reverseCost_ = target->getAdmissibleCostToGo();
                    }

                    // Update any edge in the forward queue affected by the target.
                    for (const auto &queueSource : target->getSourcesOfIncomingEdgesInForwardQueue())
                    {
                        forwardQueue_->updateIfExists({queueSource, target});
                    }

                    // Expand the target state into the reverse queue.
                    reverseQueue_->insertOrUpdate(expand(target));
                }
            }
        }

        void EITstar::improveApproximation(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // Add new states, also prunes states if enabled. The method returns true if all states have been added.
            if (graph_.addStates(batchSize_, terminationCondition))
            {
                // Reset the reverse collision detection.
                numSparseCollisionChecksCurrentLevel_ = initialNumSparseCollisionChecks_;
                numSparseCollisionChecksPreviousLevel_ = 0u;

                // Restart the reverse search.
                restartReverseSearch();

                // Reinitialize the forward queue.
                forwardQueue_->clear();
                expandStartVerticesIntoForwardQueue();
            }
        }

        ompl::base::PlannerStatus::StatusType EITstar::ensureSetup() const
        {
            // Call the base planners validity check. This checks if the
            // planner is setup if not then it calls setup().
            checkValidity();

            // Ensure the planner is setup.
            if (!setup_)
            {
                OMPL_ERROR("%s: Called solve without setting up the planner first.", name_.c_str());
                return ompl::base::PlannerStatus::StatusType::ABORT;
            }

            // Ensure the space is setup.
            if (!spaceInfo_->isSetup())
            {
                OMPL_ERROR("%s: Called solve without setting up the state space first.", name_.c_str());
                return ompl::base::PlannerStatus::StatusType::ABORT;
            }

            return ompl::base::PlannerStatus::StatusType::UNKNOWN;
        }

        ompl::base::PlannerStatus::StatusType
        EITstar::ensureStartAndGoalStates(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // If the graph currently does not have a start state, try to get one.
            if (!graph_.hasAStartState())
            {
                graph_.updateStartAndGoalStates(terminationCondition, &pis_);

                // If we could not get a start state, then there's nothing to solve.
                if (!graph_.hasAStartState())
                {
                    OMPL_WARN("%s: No solution can be found as no start states are available", name_.c_str());
                    return ompl::base::PlannerStatus::StatusType::INVALID_START;
                }
            }

            // If the graph currently does not have a goal state, we wait until we get one.
            if (!graph_.hasAGoalState())
            {
                graph_.updateStartAndGoalStates(terminationCondition, &pis_);

                // If the graph still doesn't have a goal after waiting, then there's nothing to solve.
                if (!graph_.hasAGoalState())
                {
                    OMPL_WARN("%s: No solution can be found as no goal states are available", name_.c_str());
                    return ompl::base::PlannerStatus::StatusType::INVALID_GOAL;
                }
            }

            // Would it be worth implementing a 'setup' or 'checked' status type?
            return ompl::base::PlannerStatus::StatusType::UNKNOWN;
        }

        ompl::base::PlannerStatus::StatusType EITstar::updateStatus()
        {
            // Return the appropriate planner status.
            if (objective_->isFinite(solutionCost_))
            {
                return ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION;
            }
            else if (trackApproximateSolutions_)
            {
                return ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION;
            }
            else
            {
                return ompl::base::PlannerStatus::StatusType::TIMEOUT;
            }
        }

        bool EITstar::continueReverseSearch() const
        {
            // Never continue the reverse search if the reverse queue is empty.
            if (reverseQueue_->empty())
            {
                return false;
            }
            // Always continue the reverse search if the reverse queue is not empty but the forward queue is.
            else if (forwardQueue_->empty())
            {
                return true;
            }

            /*

            There are three conditions under which the reverse search can be suspended:

               1. The best edge in the forward search has a closed target (admissible cost-to-go estimate), and the
                  reverse search cannot lead to a better solution than the potential solution of this edge.

               2. All edges in the forward queue have closed targets (admissible cost-to-go estimates).

               3. We do not care about solution cost and the least-effort edge in the forward queue is connected to the
                  reverse tree.

             */

            const bool condition1 = isClosed(forwardQueue_->peek(suboptimalityFactor_).target->asReverseVertex()) &&
                                    isBetter(forwardQueue_->getLowerBoundOnOptimalSolutionCost(),
                                             reverseQueue_->getLowerBoundOnOptimalSolutionCost());

            const bool condition2 = !forwardQueue_->containsOpenTargets(reverseSearchTag_);

            const bool condition3 = !std::isfinite(suboptimalityFactor_) &&
                                    forwardQueue_->peek(suboptimalityFactor_).target->hasReverseVertex();

            // The reverse search must be continued if it cannot be suspended.
            return !(condition1 || condition2 || condition3);
        }

        bool EITstar::continueForwardSearch() const
        {
            // Never continue to forward search if the forward queue is empty.
            if (forwardQueue_->empty())
            {
                return false;
            }

            // The forward search must be continued if the potential solution cost of the best edge is lower than the
            // current solution cost.
            return isBetter(forwardQueue_->getLowerBoundOnOptimalSolutionCost(), solutionCost_);
        }

        void EITstar::updateExactSolution()
        {
            for (const auto &goal : graph_.getGoalStates())
            {
                if (goal->hasForwardVertex())
                {
                    updateExactSolution(goal);
                }
            }
        }

        ompl::base::PlannerStatus::StatusType EITstar::updateSolution()
        {
            updateExactSolution();
            if (objective_->isFinite(solutionCost_))
            {
                return ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION;
            }
            else if (trackApproximateSolutions_)
            {
                updateApproximateSolution();
                return ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION;
            }
            else
            {
                return ompl::base::PlannerStatus::StatusType::TIMEOUT;
            }
        }

        void EITstar::restartReverseSearch()
        {
            reverseQueue_->clear();
            goalVertices_.clear();
            reverseCost_ = objective_->infiniteCost();
            for (const auto &goal : graph_.getGoalStates())
            {
                goalVertices_.emplace_back(goal->asReverseVertex());
                goal->setAdmissibleCostToGo(objective_->identityCost());
                goal->setEstimatedCostToGo(objective_->identityCost());
                goal->setEstimatedEffortToGo(0u);
            }
            expandGoalVerticesIntoReverseQueue();
            ++reverseSearchTag_;
        }

        void EITstar::updateApproximateSolution()
        {
            for (const auto &start : graph_.getStartStates())
            {
                start->asForwardVertex()->callOnBranch(
                    [this](const std::shared_ptr<eitstar::State> &state) -> void { updateApproximateSolution(state); });
            }
        }

        void EITstar::updateCurrentCostToCome(const std::shared_ptr<eitstar::State> &state)
        {
            // There is no updating to do if the state is a start.
            if (graph_.isStart(state))
            {
                return;
            }

            // If the state is not in the forward tree, then its current cost to come is infinity.
            if (!state->hasForwardVertex())
            {
                state->setCurrentCostToCome(objective_->infiniteCost());
                return;
            }

            // If the state is in the forward tree and not the start, then update its cost.
            auto forwardVertex = state->asForwardVertex();
            state->setCurrentCostToCome(combine(forwardVertex->getParent().lock()->getState()->getCurrentCostToCome(),
                                                forwardVertex->getEdgeCost()));
        }

        void EITstar::updateExactSolution(const std::shared_ptr<eitstar::State> &goal)
        {
            // Throw if the reverse root does not have a forward vertex.
            assert(goal->hasForwardVertex());

            // We update the current goal if
            //   1. We currently don't have a goal; or
            //   2. The new goal has a better cost to come than the old goal
            if (isBetter(goal->getCurrentCostToCome(), solutionCost_))
            {
                // Update the best cost.
                solutionCost_ = goal->getCurrentCostToCome();

                // Allocate the path.
                auto path = std::make_shared<ompl::geometric::PathGeometric>(spaceInfo_);

                // Allocate a vector for states. The append function of the path inserts states in front of an
                // std::vector, which is not very efficient. I'll rather iterate over the vector in reverse.
                std::vector<std::shared_ptr<State>> states;
                auto current = goal;

                // Collect all states in reverse order of the path (starting from the goal).
                while (!graph_.isStart(current))
                {
                    assert(current->asForwardVertex()->getParent().lock());
                    states.emplace_back(current);
                    current = current->asForwardVertex()->getParent().lock()->getState();
                }
                states.emplace_back(current);

                // Append all states to the path in correct order (starting from the start).
                for (auto it = states.crbegin(); it != states.crend(); ++it)
                {
                    assert(*it);
                    assert((*it)->raw());
                    path->append((*it)->raw());
                }

                // Register this solution with the problem definition.
                ompl::base::PlannerSolution solution(path);
                solution.setPlannerName(name_);
                solution.setOptimized(objective_, solutionCost_, objective_->isSatisfied(solutionCost_));
                problem_->addSolutionPath(solution);

                // If we found this solution with a suboptimality factor greater than 1, set the factor to one now.
                // Empirically, this results in faster convergence, see associated publication for more info.
                if (suboptimalityFactor_ > 1.0)
                {
                    suboptimalityFactor_ = 1.0;
                }
            }
        }

        void EITstar::updateApproximateSolution(const std::shared_ptr<eitstar::State> &state)
        {
            assert(trackApproximateSolutions_);
            if (state->hasForwardVertex() || graph_.isStart(state))
            {
                const auto costToGoal = computeCostToGoToGoal(state);
                if (isBetter(costToGoal, approximateSolutionCostToGoal_))
                {
                    approximateSolutionCost_ = state->getCurrentCostToCome();
                    approximateSolutionCostToGoal_ = costToGoal;
                }
            }
        }

        ompl::base::Cost EITstar::computeCostToGoToGoal(const std::shared_ptr<eitstar::State> &state) const
        {
            auto bestCost = objective_->infiniteCost();
            for (const auto &goal : graph_.getGoalStates())
            {
                bestCost = objective_->betterCost(bestCost, objective_->motionCost(state->raw(), goal->raw()));
            }
            return bestCost;
        }

        void EITstar::informAboutNewSolution() const
        {
            OMPL_INFORM("%s (%u iterations): Found a new exact solution of cost %.4f. Sampled a total of %u states, %u "
                        "of which were valid samples (%.1f \%). Processed %u edges, %u of which were collision checked "
                        "(%.1f \%). The forward search tree has %u vertices. The reverse search tree has %u vertices.",
                        name_.c_str(), iteration_, solutionCost_.value(), graph_.getNumberOfSampledStates(),
                        graph_.getNumberOfValidSamples(),
                        graph_.getNumberOfSampledStates() == 0u ?
                            0.0 :
                            100.0 * (static_cast<double>(graph_.getNumberOfValidSamples()) /
                                     static_cast<double>(graph_.getNumberOfSampledStates())),
                        numProcessedEdges_, numCollisionCheckedEdges_,
                        numProcessedEdges_ == 0u ? 0.0 :
                                                   100.0 * (static_cast<float>(numCollisionCheckedEdges_) /
                                                            static_cast<float>(numProcessedEdges_)),
                        countNumVerticesInForwardTree(), countNumVerticesInReverseTree());
        }

        void EITstar::informAboutPlannerStatus(ompl::base::PlannerStatus::StatusType status) const
        {
            switch (status)
            {
                case ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION:
                {
                    OMPL_INFORM("%s (%u iterations): Found an exact solution of cost %.4f.", name_.c_str(), iteration_,
                                solutionCost_.value());
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION:
                {
                    OMPL_INFORM("%s (%u iterations): Did not find an exact solution, but found an approximate solution "
                                "of cost %.4f which is %.4f away from a goal (in cost space).",
                                name_.c_str(), iteration_, approximateSolutionCost_.value(),
                                approximateSolutionCostToGoal_.value());
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::TIMEOUT:
                {
                    if (trackApproximateSolutions_)
                    {
                        OMPL_INFORM("%s (%u iterations): Did not find any solution.", name_.c_str(), iteration_);
                    }
                    else
                    {
                        OMPL_INFORM("%s (%u iterations): Did not find an exact solution, and tracking approximate "
                                    "solutions is disabled.",
                                    name_.c_str(), iteration_);
                    }
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::UNKNOWN:
                case ompl::base::PlannerStatus::StatusType::INVALID_START:
                case ompl::base::PlannerStatus::StatusType::INVALID_GOAL:
                case ompl::base::PlannerStatus::StatusType::UNRECOGNIZED_GOAL_TYPE:
                case ompl::base::PlannerStatus::StatusType::CRASH:
                case ompl::base::PlannerStatus::StatusType::ABORT:
                case ompl::base::PlannerStatus::StatusType::TYPE_COUNT:
                {
                    OMPL_INFORM("%s (%u iterations): Unable to solve the given planning problem.", name_.c_str(),
                                iteration_);
                }
            }
        }

        unsigned int EITstar::countNumVerticesInForwardTree() const
        {
            const auto states = graph_.getStates();
            return std::count_if(states.cbegin(), states.cend(),
                                 [](const auto &state) { return state->hasForwardVertex(); });
        }

        unsigned int EITstar::countNumVerticesInReverseTree() const
        {
            const auto states = graph_.getStates();
            return std::count_if(states.cbegin(), states.cend(),
                                 [](const auto &state) { return state->hasReverseVertex(); });
        }

        bool EITstar::couldImproveForwardPath(const Edge &edge) const
        {
            // If we currently don't have a solution, the anwer is yes.
            if (!objective_->isFinite(solutionCost_))
            {
                // Compare the costs of the full path heuristic with the current cost of the start state.
                const auto heuristicPathCost =
                    combine(edge.source->getCurrentCostToCome(),
                            objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw()),
                            objective_->costToGo(edge.target->raw(), problem_->getGoal().get()));
                if (isBetter(heuristicPathCost, solutionCost_))
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return true;
            }
        }

        bool EITstar::couldImproveForwardTree(const Edge &edge) const
        {
            const auto heuristicCostToCome =
                combine(edge.source->getCurrentCostToCome(),
                        objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw()));
            return isBetter(heuristicCostToCome, edge.target->getCurrentCostToCome());
        }

        bool EITstar::doesImproveForwardPath(const Edge &edge, const ompl::base::Cost &edgeCost) const
        {
            // If we don't have a solution yet, the answer is imediately true.
            if (!objective_->isFinite(solutionCost_))
            {
                return true;
            }

            // Check whether it can improve the current solution.
            return isBetter(
                combine(edge.source->getCurrentCostToCome(), edgeCost, edge.target->getLowerBoundCostToGo()),
                solutionCost_);
        }

        bool EITstar::doesImproveForwardTree(const Edge &edge, const ompl::base::Cost &edgeCost) const
        {
            return isBetter(combine(edge.source->getCurrentCostToCome(), edgeCost),
                            edge.target->getCurrentCostToCome());
        }

        ompl::base::Cost EITstar::estimateCostToTarget(const eitstar::Edge &edge) const
        {
            return combine(edge.source->getEstimatedCostToGo(),
                           objective_->motionCostBestEstimate(edge.source->raw(), edge.target->raw()));
        }

        unsigned int EITstar::estimateEffortToTarget(const eitstar::Edge &edge) const
        {
            return edge.source->getEstimatedEffortToGo() +
                   space_->validSegmentCount(edge.target->raw(), edge.source->raw());
        }

        bool EITstar::isValid(const Edge &edge) const
        {
            // The number of checks required to determine whether the edge is valid is the valid segment count minus one
            // because we know that the source and target states are valid.
            const std::size_t numChecks = space_->validSegmentCount(edge.source->raw(), edge.target->raw()) - 1u;
            return isValidAtResolution(edge, numChecks);
        }

        bool EITstar::couldBeValid(const Edge &edge) const
        {
            return isValidAtResolution(edge, numSparseCollisionChecksCurrentLevel_);
        }

        bool EITstar::isValidAtResolution(const Edge &edge, std::size_t numChecks) const
        {
            // Check if the edge is whitelisted.
            if (edge.source->isWhitelisted(edge.target))
            {
                return true;
            }

            // If the edge is blacklisted.
            if (edge.source->isBlacklisted(edge.target))
            {
                return false;
            }

            // Get the segment count for the full resolution.
            const std::size_t fullSegmentCount = space_->validSegmentCount(edge.source->raw(), edge.target->raw());

            // The segment count is the number of checks on this level plus 1, capped by the full resolution segment
            // count.
            const auto segmentCount = std::min(numChecks + 1u, fullSegmentCount);

            /***
               Let's say we want to perform seven collision checks on an edge:

               position of checks: |--------x--------x--------x--------x--------x--------x--------x--------|
               indices of checks:           1        2        3        4        5        6        7
               order of testing:            4        2        5        1        6        3        7

               We create a queue that holds segments and always test the midpoint of the segments. We start
               with the outermost indices and then break the segment in half until the segment collapses to a single
               point:

                 1. indices = { (1, 7) }
                    current = (1, 7) -> test midpoint = 4 -> add (1, 3) and (5, 7) to queue

                 2. indices = { (1, 3), (5, 7) }
                    current (1, 3) -> test midpoint = 2 -> add (1, 1) and (3, 3) to queue

                 3. indices = { (5, 7), (1, 1), (3, 3) }
                    current (5, 7) -> test midpoint = 6 -> add (5, 5) and (7, 7) to queue

                 4. indices = { (1, 1), (3, 3), (5, 5), (7, 7) }
                    current (1, 1) -> test midpoint = 1 -> add nothing to the queue

                 5. indices = { (3, 3), (5, 5), (7, 7) }
                    current (3, 3) -> test midpoint = 3 -> add nothing to the queue

                 6. indices = { (5, 5) (7, 7) }
                    current (5, 5) -> test midpoint = 5 -> add nothing to the queue

                 7. indices = { (7, 7) }
                    current (7, 7) -> test midpoint = 7 -> add nothing to the queue
            ***/

            // Store the current check number.
            std::size_t currentCheck = 1u;

            // Get the number of checks already performed on this edge.
            const std::size_t performedChecks = edge.target->getIncomingCollisionCheckResolution(edge.source);

            // Initialize the queue of positions to be tested.
            std::queue<std::pair<std::size_t, std::size_t>> indices;
            indices.emplace(1u, numChecks);

            // Test states while there are states to be tested.
            while (!indices.empty())
            {
                // Get the current segment and remove if from the queue.
                const auto current = indices.front();
                indices.pop();

                // Get the midpoint of the segment.
                auto mid = (current.first + current.second) / 2;

                // Create the first half of the split segment if necessary.
                if (current.first < mid)
                {
                    indices.emplace(current.first, mid - 1u);
                }

                // Create the second half of the split segment if necessary.
                if (current.second > mid)
                {
                    indices.emplace(mid + 1u, current.second);
                }

                // Only do the detection if we haven't tested this state on a previous level.
                if (currentCheck > performedChecks)
                {
                    space_->interpolate(edge.source->raw(), edge.target->raw(),
                                        static_cast<double>(mid) / static_cast<double>(segmentCount), detectionState_);

                    if (!spaceInfo_->isValid(detectionState_))
                    {
                        // Blacklist the edge.
                        edge.source->blacklist(edge.target);
                        edge.target->blacklist(edge.source);

                        // Register it with the graph.
                        graph_.registerInvalidEdge(edge);
                        return false;
                    }
                }

                // Increase the current check number.
                ++currentCheck;
            }

            // Remember at what resolution this edge was already checked. We're assuming that the number of collision
            // checks is symmetric for each edge.
            edge.source->setIncomingCollisionCheckResolution(edge.target, currentCheck);
            edge.target->setIncomingCollisionCheckResolution(edge.source, currentCheck);

            // Whitelist this edge if it was checked at full resolution.
            if (segmentCount == fullSegmentCount)
            {
                ++numCollisionCheckedEdges_;
                edge.source->whitelist(edge.target);
                edge.target->whitelist(edge.source);
            }

            return true;
        }

        bool EITstar::isBetter(const ompl::base::Cost &lhs, const ompl::base::Cost &rhs) const
        {
            return objective_->isCostBetterThan(lhs, rhs);
        }

        ompl::base::Cost EITstar::combine(const ompl::base::Cost &lhs, const ompl::base::Cost &rhs) const
        {
            return objective_->combineCosts(lhs, rhs);
        }

        void EITstar::expandStartVerticesIntoForwardQueue()
        {
            for (auto &vertex : startVertices_)
            {
                forwardQueue_->insertOrUpdate(expand(vertex->getState()));
            }
        }

        void EITstar::expandGoalVerticesIntoReverseQueue()
        {
            for (auto &vertex : goalVertices_)
            {
                reverseQueue_->insertOrUpdate(expand(vertex->getState()));
            }
        }

        std::tuple<std::shared_ptr<eitstar::State>, ompl::base::Cost, ompl::base::Cost>
        EITstar::getBestParentInReverseTree(const std::shared_ptr<eitstar::State> &state) const
        {
            std::shared_ptr<eitstar::State> bestParent;
            ompl::base::Cost bestCost = objective_->infiniteCost();
            ompl::base::Cost bestEdgeCost = objective_->infiniteCost();
            for (const auto &neighbor : graph_.getNeighbors(state))
            {
                if (neighbor->hasReverseVertex())
                {
                    const auto neighborEdgeCost = objective_->motionCostBestEstimate(neighbor->raw(), state->raw());
                    const auto neighborCost = combine(neighbor->getAdmissibleCostToGo(), neighborEdgeCost);
                    if (isBetter(neighborCost, bestCost))
                    {
                        bestParent = neighbor;
                        bestCost = neighborCost;
                        bestEdgeCost = neighborEdgeCost;
                    }
                }
            }
            return {bestParent, bestCost, bestEdgeCost};
        }

        bool EITstar::isClosed(const std::shared_ptr<Vertex> &vertex) const
        {
            return vertex->getExpandTag() == reverseSearchTag_;
        }

        bool EITstar::isInForwardTree(const Edge &edge) const
        {
            if (!edge.source->hasForwardVertex() || !edge.target->hasForwardVertex())
            {
                return false;
            }

            return edge.target->asForwardVertex()->isParent(edge.source->asForwardVertex());
        }

        bool EITstar::isInReverseTree(const Edge &edge) const
        {
            if (!edge.source->hasReverseVertex() || !edge.target->hasReverseVertex())
            {
                return false;
            }

            return edge.target->asReverseVertex()->isParent(edge.source->asReverseVertex());
        }

        bool EITstar::doesImproveReversePath(const Edge &edge) const
        {
            // If there is no reverse path the answer is ja.
            if (!objective_->isFinite(reverseCost_))
            {
                return true;
            }

            // Compare the costs of the full path heuristic with the current cost of the start state.
            const auto heuristicPathCost =
                combine(edge.source->getAdmissibleCostToGo(),
                        objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw()),
                        edge.target->getLowerBoundCostToCome());

            return isBetter(heuristicPathCost, reverseCost_);
        }

        bool EITstar::doesImproveReverseTree(const Edge &edge, const ompl::base::Cost &admissibleEdgeCost) const
        {
            return isBetter(combine(edge.source->getAdmissibleCostToGo(), admissibleEdgeCost),
                            edge.target->getAdmissibleCostToGo());
        }

        std::vector<Edge> EITstar::expandUnlessGoal(const std::shared_ptr<State> &state) const
        {
            if (graph_.isGoal(state))
            {
                return {};
            }

            return expand(state);
        }

        std::vector<Edge> EITstar::expand(const std::shared_ptr<State> &state) const
        {
            // Only states associated with a vertex in either of the trees should be expanded.
            assert(state->hasForwardVertex() || state->hasReverseVertex());

            // Prepare the return variable.
            std::vector<Edge> outgoingEdges;

            // Get the neighbors in the current graph.
            for (const auto &neighborState : graph_.getNeighbors(state))
            {
                outgoingEdges.emplace_back(state, neighborState);
            }

            // If the state is in the forward search tree, extra edges have to be added.
            if (state->hasForwardVertex())
            {
                // Get the vertex in the forward search tree associated with this state.
                auto forwardVertex = state->asForwardVertex();

                // Add the outgoing edge to this vertex's parent in the forward tree, if it exists.
                if (!graph_.isStart(state))
                {
                    // If this vertex is not a start state, it must have a parent.
                    assert(forwardVertex->getParent().lock());

                    // Get the state associated with the parent vertex.
                    auto forwardParentState = forwardVertex->getParent().lock()->getState();

                    // Add the edge to the forward tree parent if it has not already being added.
                    if (std::find_if(outgoingEdges.cbegin(), outgoingEdges.cend(),
                                     [&forwardParentState](const auto &edge) {
                                         return edge.target->getId() == forwardParentState->getId();
                                     }) == outgoingEdges.cend())
                    {
                        outgoingEdges.emplace_back(state, forwardParentState);
                    }
                }

                // Add the edge to the forward children.
                for (const auto &child : forwardVertex->getChildren())
                {
                    // Get the state associated with the child vertex.
                    auto forwardChildState = child->getState();

                    // Add the edge to the forward tree child if it has not already being added.
                    if (std::find_if(outgoingEdges.cbegin(), outgoingEdges.cend(),
                                     [&forwardChildState](const auto &edge) {
                                         return edge.target->getId() == forwardChildState->getId();
                                     }) == outgoingEdges.cend())
                    {
                        outgoingEdges.emplace_back(state, forwardChildState);
                    }
                }
            }

            // If the state is in the reverse search tree, extra edges have to be added.
            if (state->hasReverseVertex())
            {
                // Get the vertex in the reverse search tree associated with this state.
                auto reverseVertex = state->asReverseVertex();

                // Add the outgoing edge to this vertex's parent in the reverse tree, if it exists.
                if (!graph_.isGoal(state))
                {
                    // If this state is not a goal, it must have a parent.
                    assert(reverseVertex->getParent().lock());

                    // Get the state associated with the parent vertex.
                    auto reverseParentState = reverseVertex->getParent().lock()->getState();

                    // Add the edge to the reverse tree parent if it has not already being added.
                    if (std::find_if(outgoingEdges.cbegin(), outgoingEdges.cend(),
                                     [&reverseParentState](const auto &edge) {
                                         return edge.target->getId() == reverseParentState->getId();
                                     }) == outgoingEdges.cend())
                    {
                        outgoingEdges.emplace_back(state, reverseParentState);
                    }
                }

                // Add the edge to the reverse children.
                for (const auto &child : reverseVertex->getChildren())
                {
                    // Get the state associated with the child vertex.
                    auto reverseChildState = child->getState();

                    // Add the edge to the reverse tree parent if it has not already being added.
                    if (std::find_if(outgoingEdges.cbegin(), outgoingEdges.cend(),
                                     [&reverseChildState](const auto &edge) {
                                         return edge.target->getId() == reverseChildState->getId();
                                     }) == outgoingEdges.cend())
                    {
                        outgoingEdges.emplace_back(state, reverseChildState);
                    }
                }
            }

            return outgoingEdges;
        }

        std::vector<eitstar::Edge> EITstar::expandForwardRootsInReverseTree() const
        {
            std::vector<eitstar::Edge> allEdges;
            for (const auto root : startVertices_)
            {
                if (root->getTwin().lock())
                {
                    auto edges = expand(root->getState());
                    allEdges.insert(allEdges.end(), edges.begin(), edges.end());
                }
            }
            return allEdges;
        }
    }  // namespace geometric
}  // namespace ompl

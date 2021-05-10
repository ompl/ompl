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

                // Initialize the solution cost to infinity.
                solutionCost_ = objective_->infiniteCost();

                // Initialize the cost of the best reverse path to infinity.
                reverseCost_ = objective_->infiniteCost();

                // Setup the graph with the problem information.
                graph_.setup(problem_, &pis_);

                // Set the best cost to infinity.
                solutionCost_ = objective_->infiniteCost();

                // Instantiate the queues.
                forwardQueue_ = std::make_unique<eitstar::ForwardQueue>(objective_, space_);
                reverseQueue_ = std::make_unique<eitstar::ReverseQueue>(objective_, space_);

                // Create the forward roots.
                for (const auto &start : graph_.getStartStates())
                {
                    start->setCurrentCostToCome(objective_->identityCost());
                    start->setAdmissibleCostToGo(objective_->infiniteCost());
                    start->setEstimatedCostToGo(objective_->infiniteCost());
                    start->setEstimatedEffortToGo(std::numeric_limits<std::size_t>::max());
                    forwardRoots_.emplace_back(start->asForwardVertex());
                }

                // Create the reverse roots.
                for (const auto &goal : graph_.getGoalStates())
                {
                    goal->setCurrentCostToCome(objective_->infiniteCost());
                    goal->setAdmissibleCostToGo(objective_->identityCost());
                    goal->setEstimatedCostToGo(objective_->identityCost());
                    goal->setEstimatedEffortToGo(0u);
                    reverseRoots_.emplace_back(goal->asReverseVertex());
                }
            }
            else
            {
                setup_ = false;
                OMPL_WARN("EIT*: Unable to setup without a problem definition.");
            }
        }

        ompl::base::PlannerStatus EITstar::solve(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // The planner status to return.
            auto status = ompl::base::PlannerStatus::StatusType::UNKNOWN;

            // Make sure everything is setup.
            if (!setup_)
            {
                throw std::runtime_error("Called solve on EIT* without setting up the planner first.");
            }
            if (!spaceInfo_->isSetup())
            {
                throw std::runtime_error("Called solve on EIT* without setting up the state space first.");
            }

            // If the graph currently does not have a goal state, we wait until we get one.
            if (!graph_.hasGoalState())
            {
                graph_.updateStartAndGoalStates(terminationCondition, &pis_);
            }

            if (!graph_.hasStartState())
            {
                OMPL_WARN("%s: No solution can be found as no start states are available", name_.c_str());
                status = ompl::base::PlannerStatus::StatusType::INVALID_START;
                informAboutPlannerStatus(status);
                return status;
            }

            // If the graph still doesn't have a goal after waiting there's nothing to solve.
            if (!graph_.hasGoalState())
            {
                OMPL_WARN("%s: No solution can be found as no goal states are available", name_.c_str());
                status = ompl::base::PlannerStatus::StatusType::INVALID_GOAL;
                informAboutPlannerStatus(status);
                return status;
            }

            OMPL_INFORM("%s: Searching for a solution to the given planning problem. The current best solution cost is "
                        "%.4f",
                        name_.c_str(), solutionCost_.value());

            // Iterate until stopped or objective is satisfied.
            while (!terminationCondition && !objective_->isSatisfied(solutionCost_))
            {
                iterate(terminationCondition);
            }

            // Someone might call ProblemDefinition::clearSolutionPaths() between invocations of Planner::sovle(), in
            // which case previously found solutions are not registered with the problem definition anymore.
            updateExactSolution();

            // If there are no exact solutions registered in the problem definition and we're tracking approximate
            // solutions, find the best vertex in the graph.
            if (!pdef_->hasExactSolution() && trackApproximateSolutions_)
            {
                updateApproximateSolution();
            }

            // Return the appropriate planner status.
            if (objective_->isFinite(solutionCost_))
            {
                return ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION;
            }
            else
            {
                return ompl::base::PlannerStatus::StatusType::TIMEOUT;
            }
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

        void EITstar::enableCollisionDetectionInReverseSearch(bool enable)
        {
            isCollisionDetectionInReverseTreeEnabled_ = enable;
        }

        bool EITstar::isCollisionDetectionInReverseSearchEnabled() const
        {
            return isCollisionDetectionInReverseTreeEnabled_;
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

        void EITstar::resetSuboptimalityFactorOnEveryApproximation(bool reset)
        {
            resetSuboptimalityFactorOnEveryApproximation_ = reset;
        }

        bool EITstar::isSuboptimalityFactorOnEveryApproximationReset() const
        {
            return resetSuboptimalityFactorOnEveryApproximation_;
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
            for (const auto &root : reverseRoots_)
            {
                getEdgesRecursively(root);
            }

            // Return all edges in the reverse tree.
            return edges;
        }

        Edge EITstar::getNextForwardEdge() const
        {
            assert(forwardQueue_);
            return forwardQueue_->peek(suboptimalityFactor_);
        }

        Edge EITstar::getNextReverseEdge() const
        {
            assert(reverseQueue_);
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
            // If this is the first iteration, populate the reverse queue.
            if (iteration_ == 0u)
            {
                expandReverseRootsIntoReverseQueue();
                phase_ = reverseQueue_->empty() ? Phase::IMPROVE_APPROXIMATION : Phase::REVERSE_SEARCH;
            }

            switch (phase_)
            {
                case Phase::REVERSE_SEARCH:
                {
                    iterateReverseSearch();
                    break;
                }
                case Phase::FORWARD_SEARCH:
                {
                    iterateForwardSearch();
                    break;
                }
                case Phase::IMPROVE_APPROXIMATION:
                {
                    reverseQueue_->clear();
                    forwardQueue_->clear();
                    improveApproximation(terminationCondition);
                    break;
                }
                default:
                {
                    // We should never reach here.
                    assert(false);
                }
            };

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

            // Assert that the source of the edge has a forward vertex.
            assert(edge.source->hasForwardVertex());

            // Check if this edge can possibly improve the forward path.
            if (couldImproveForwardPath(edge))
            {
                // Check if the edge's parent is already the parent of the child.
                if (auto currentParent = edge.target->asForwardVertex()->getParent().lock())
                {
                    if (currentParent->getId() == edge.source->asForwardVertex()->getId() &&
                        !graph_.isGoal(edge.target))
                    {
                        // Get the outgoing edges of the child.
                        jitSearchEdgeCache_ = expand(edge.target);

                        // Insert them if they can be inserted.
                        if (canBeInsertedInForwardQueue(jitSearchEdgeCache_))
                        {
                            forwardQueue_->insert(jitSearchEdgeCache_);
                            jitSearchEdgeCache_.clear();
                        }
                        else
                        {
                            if (reverseQueue_->empty())
                            {
                                // Try each edge individually.
                                for (const auto &edge : jitSearchEdgeCache_)
                                {
                                    if (doAllVerticesHaveAdmissibleCostToGo(edge))
                                    {
                                        forwardQueue_->insert(edge);
                                    }
                                }

                                // Clear all edges. The ones that are not inserted are between vertices that are not in
                                // the same connected component.
                                jitSearchEdgeCache_.clear();

                                // Set the phase. If no edges were inserted, improve the approximation.
                                phase_ = forwardQueue_->empty() ? Phase::IMPROVE_APPROXIMATION : Phase::FORWARD_SEARCH;
                            }
                        }
                    }
                }
                // Check if it can possibly improve the tree.
                if (couldImproveForwardTree(edge))
                {
                    // Check if the edge is valid.
                    if (isValid(edge))
                    {
                        // Compute the true edge cost.
                        auto trueEdgeCost = objective_->motionCost(edge.source->raw(), edge.target->raw());

                        // Compute the true cost to come to the target through this edge.
                        auto trueCostThroughEdge =
                            objective_->combineCosts(edge.source->getCurrentCostToCome(), trueEdgeCost);

                        // Check if the edge can actually improve the forward path and tree.
                        if (objective_->isCostBetterThan(trueCostThroughEdge, edge.target->getCurrentCostToCome()) &&
                            objective_->isCostBetterThan(
                                objective_->combineCosts(trueCostThroughEdge, edge.target->getAdmissibleCostToGo()),
                                solutionCost_))
                        {
                            // Convenience access to parent and child vertices.
                            auto parentVertex = edge.source->asForwardVertex();
                            auto childVertex = edge.target->asForwardVertex();

                            // Update the parent of the child in the forward tree.
                            childVertex->updateParent(parentVertex);

                            // Add the child to the parents children.
                            parentVertex->addChild(childVertex);

                            // Set the edge cost associated with this parent.
                            childVertex->setEdgeCost(trueEdgeCost);

                            // Update the cost-to-come.
                            edge.target->setCurrentCostToCome(trueCostThroughEdge);

                            // Update the cost of the children.
                            auto changedVertices = childVertex->updateCurrentCostOfChildren(objective_);

                            // Update the edges in the queue.
                            for (const auto &vertex : changedVertices)
                            {
                                forwardQueue_->update({vertex->getParent().lock()->getState(), vertex->getState()});

                                // Update the solution if the vertex is a goal.
                                if (graph_.isGoal(vertex->getState()))
                                {
                                    updateExactSolution(vertex->getState());
                                }
                            }

                            // Expand the outgoing edges into the queue unless this state is the goal state.
                            if (!graph_.isGoal(edge.target))
                            {
                                // Get the outgoing edges of the child.
                                jitSearchEdgeCache_ = expand(edge.target);

                                // Insert them if they can be inserted.
                                if (canBeInsertedInForwardQueue(jitSearchEdgeCache_))
                                {
                                    forwardQueue_->insert(jitSearchEdgeCache_);
                                    jitSearchEdgeCache_.clear();
                                }
                                else
                                {
                                    if (!reverseQueue_->empty())
                                    {
                                        phase_ = Phase::REVERSE_SEARCH;
                                    }
                                    else
                                    {
                                        for (const auto &edge : jitSearchEdgeCache_)
                                        {
                                            if (doAllVerticesHaveAdmissibleCostToGo(edge))
                                            {
                                                forwardQueue_->insert(edge);
                                            }
                                        }
                                        phase_ = forwardQueue_->empty() ? Phase::IMPROVE_APPROXIMATION :
                                                                          Phase::FORWARD_SEARCH;
                                    }
                                }
                            }
                            else  // It is the goal state, update the solution.
                            {
                                updateExactSolution(edge.target);
                            }
                        }
                    }
                    else
                    {
                        // Assert the edge is actually invalid.
                        assert(!motionValidator_->checkMotion(edge.source->raw(), edge.target->raw()));

                        // Check if the edge is used in the reverse tree.
                        bool isSourceInvalidated =
                            static_cast<bool>(edge.source->asReverseVertex()->getParent().lock()) &&
                            edge.source->asReverseVertex()->getParent().lock()->getId() ==
                                edge.target->asReverseVertex()->getId();
                        bool isTargetInvalidated =
                            static_cast<bool>(edge.target->asReverseVertex()->getParent().lock()) &&
                            edge.target->asReverseVertex()->getParent().lock()->getId() ==
                                edge.source->asReverseVertex()->getId();

                        // If this edge was in the reverse search tree, it could be updated.
                        if (isSourceInvalidated || isTargetInvalidated)
                        {
                            // Repair the reverse search tree if desired.
                            if (isCollisionDetectionInReverseTreeEnabled_)
                            {
                                increaseSparseCollisionDetectionResolutionAndRestartReverseSearch();
                            }
                        }
                    }
                }
            }

            // Clear the queue if no edge in it can possibly improve the current solution.
            if (objective_->isCostBetterThan(solutionCost_, forwardQueue_->getLowerBoundOnOptimalSolutionCost()))
            {
                forwardQueue_->clear();
            }

            // If the forward queue is empty, move on to the next phase.
            if (phase_ == Phase::FORWARD_SEARCH && forwardQueue_->empty())
            {
                phase_ = Phase::IMPROVE_APPROXIMATION;
            }
        }

        void EITstar::iterateReverseSearch()
        {
            // Ensure the reverse queue is not empty.
            assert(!reverseQueue_->empty());

            // Get the top edge from the queue.
            auto edge = reverseQueue_->peek();

            // The parent vertex must have an associated vertex in the tree.
            assert(edge.source->hasReverseVertex());

            // Check whether we can suspend the reverse search.
            if (jitSearchEdgeCache_.empty() && !doesImproveReversePath(edge))
            {
                // If the forward roots haven't been expanded on this RGG yet, expand them now.
                if (startExpansionGraphTag_ != graph_.getTag())
                {
                    // Expand the forward roots that are in the reverse tree.
                    jitSearchEdgeCache_ = expandForwardRootsInReverseTree();
                    updateJitSearchEdgeCache();

                    if (jitSearchEdgeCache_.empty())
                    {
                        // Set the phase. If the insert resulted in zero edges, then improve the approximation.
                        phase_ = forwardQueue_->empty() ? Phase::IMPROVE_APPROXIMATION : Phase::FORWARD_SEARCH;
                    }

                    // Register the expansion of the start.
                    startExpansionGraphTag_ = graph_.getTag();
                }
                else
                {
                    // Rebuild the forward queue, as the reverse search might have updated the used heuristics.
                    forwardQueue_->rebuild();

                    // If the forward queue is empty now, we're done with this batch because all edges that were in
                    // the queue were invalidated by repairing the reverse search.
                    phase_ = forwardQueue_->empty() ? Phase::IMPROVE_APPROXIMATION : Phase::FORWARD_SEARCH;
                }

                // We're done with the reverse search for now.
                return;
            }

            // This edge needs to be processed, pop it from the queue.
            reverseQueue_->pop();

            // Register the expansion of its parent.
            edge.source->asReverseVertex()->registerExpansionInReverseSearch(reverseSearchTag_);

            // Simply expand the child vertex if the edge is already in the reverse tree, and the child has not been
            // expanded yet.
            if (auto currentParentOfTarget = edge.target->asReverseVertex()->getParent().lock())
            {
                auto targetVertex = edge.target->asReverseVertex();

                if (currentParentOfTarget->getId() == edge.source->asReverseVertex()->getId())
                {
                    reverseQueue_->insert(expand(edge.target));
                    return;
                }
            }

            if (couldBeValid(edge))
            {
                // Compute the heuristic cost.
                const auto admissibleEdgeCost = objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw());

                // Incorporate the edge in the reverse tree if it provides an improvement.
                if (doesImproveReverseTree(edge, admissibleEdgeCost))
                {
                    // Get the parent and child vertices.
                    auto parentVertex = edge.source->asReverseVertex();
                    auto childVertex = edge.target->asReverseVertex();
                    assert(!isClosed(childVertex));

                    // Update the parent of the child in the reverse tree.
                    childVertex->updateParent(parentVertex);

                    // Add the child to the children of the parent.
                    parentVertex->addChild(childVertex);

                    // Update the admissible cost to go.
                    edge.target->setAdmissibleCostToGo(
                        objective_->combineCosts(edge.source->getAdmissibleCostToGo(), admissibleEdgeCost));

                    // Update the best cost estimate of the target state if this edge can improve it.
                    const auto bestCostEstimateThroughThisEdge = objective_->combineCosts(
                        edge.source->getEstimatedCostToGo(),
                        objective_->motionCostBestEstimate(edge.source->raw(), edge.target->raw()));
                    if (objective_->isCostBetterThan(bestCostEstimateThroughThisEdge,
                                                     edge.target->getEstimatedCostToGo()))
                    {
                        edge.target->setEstimatedCostToGo(bestCostEstimateThroughThisEdge);
                    }

                    // Update the best effort estimate of the target state if this edge can improve it.
                    const auto bestEffortEstimateThroughtThisEdge =
                        edge.source->getEstimatedEffortToGo() +
                        spaceInfo_->getStateSpace()->validSegmentCount(edge.source->raw(), edge.target->raw());
                    if (bestEffortEstimateThroughtThisEdge < edge.target->getEstimatedEffortToGo())
                    {
                        edge.target->setEstimatedEffortToGo(bestEffortEstimateThroughtThisEdge);
                    }

                    // If this edge improves the reverse cost, update it.
                    if (graph_.isStart(edge.target) &&
                        objective_->isCostBetterThan(edge.target->getAdmissibleCostToGo(), reverseCost_))
                    {
                        reverseCost_ = edge.target->getAdmissibleCostToGo();
                    }

                    // Expand the outgoing edges into the queue unless this has already happened.
                    if (!isClosed(childVertex))
                    {
                        reverseQueue_->insert(expand(edge.target));
                    }
                }
            }

            // If there are edges to be inserted in the forward queue, insert the ones that can be inserted.
            updateJitSearchEdgeCache();

            if (reverseQueue_->empty())
            {
                // If the reverse queue is empty, we can clear the jit search edge as all edges that have now been are
                // between vertices in different components of the RGG.
                jitSearchEdgeCache_.clear();

                // Insert edges into the forward queue if there could be a path.
                if (isAnyForwardRootInReverseTree())
                {
                    // If the start vertex hasn't already been expanded on this RGG, then expand it now.
                    if (startExpansionGraphTag_ != graph_.getTag())
                    {
                        // Get the outgoing edges of the child.
                        jitSearchEdgeCache_ = expandForwardRootsInReverseTree();
                        updateJitSearchEdgeCache();

                        // Clear all edges. If the reverse queue is empty, there's no way the vertices of the edges that
                        // weren't inserted above will be inserted later.
                        jitSearchEdgeCache_.clear();

                        // Register the expansion of the start.
                        startExpansionGraphTag_ = graph_.getTag();
                    }
                    else
                    {
                        // Rebuild the forward queue, as the reverse search might have updated the used heuristics.
                        forwardQueue_->rebuild();
                    }

                    // Set the phase. If the insert resulted in zero edges, then improve the approximation.
                    phase_ = forwardQueue_->empty() ? Phase::IMPROVE_APPROXIMATION : Phase::FORWARD_SEARCH;
                }
                else
                {
                    phase_ = Phase::IMPROVE_APPROXIMATION;
                }
            }
        }

        void EITstar::improveApproximation(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // Add new states, also prunes states if enabled. The method returns true if all states have been added.
            if (graph_.addStates(batchSize_, terminationCondition))
            {
                // Reset the suboptimality factor.
                if (resetSuboptimalityFactorOnEveryApproximation_)
                {
                    suboptimalityFactor_ = std::numeric_limits<double>::infinity();
                }

                // Reset the reverse collision detection.
                numSparseCollisionChecksCurrentLevel_ = initialNumSparseCollisionChecks_;
                numSparseCollisionChecksPreviousLevel_ = 0u;

                // Restart the reverse search.
                reverseRoots_.clear();
                reverseCost_ = objective_->infiniteCost();
                for (const auto &goal : graph_.getGoalStates())
                {
                    goal->setCurrentCostToCome(objective_->infiniteCost());
                    goal->setAdmissibleCostToGo(objective_->identityCost());
                    goal->setEstimatedCostToGo(objective_->identityCost());
                    goal->setEstimatedEffortToGo(0u);
                    reverseRoots_.emplace_back(goal->asReverseVertex());
                    reverseQueue_->insert(expand(goal));
                }

                // Clear the jit search edge cache.
                jitSearchEdgeCache_.clear();

                // If expanding the goal state actually produced edges, let's start the reverse search.
                // Otherwise, we stay in the improve approximation phase.
                if (!reverseQueue_->empty())
                {
                    phase_ = Phase::REVERSE_SEARCH;

                    // Update the search tag.
                    ++reverseSearchTag_;
                }
            }
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
            if (state->hasForwardVertex())
            {
                auto forwardVertex = state->asForwardVertex();
                state->setCurrentCostToCome(
                    objective_->combineCosts(forwardVertex->getParent().lock()->getState()->getCurrentCostToCome(),
                                             forwardVertex->getEdgeCost()));
            }
            else
            {
                state->setCurrentCostToCome(objective_->infiniteCost());
            }
        }

        void EITstar::updateExactSolution(const std::shared_ptr<eitstar::State> &goal)
        {
            // Throw if the reverse root does not have a forward vertex.
            assert(goal->hasForwardVertex());

            // We update the current goal if
            //   1. We currently don't have a goal; or
            //   2. The new goal has a better cost to come than the old goal
            if (objective_->isCostBetterThan(goal->getCurrentCostToCome(), solutionCost_))
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
                if (objective_->isCostBetterThan(costToGoal, approximateSolutionCostToGoal_))
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

        void EITstar::increaseSparseCollisionDetectionResolutionAndRestartReverseSearch()
        {
            // Remember the number of collision checks on the previous level.
            numSparseCollisionChecksPreviousLevel_ = numSparseCollisionChecksCurrentLevel_;

            // Increase the corresponding number of sparse checks.
            numSparseCollisionChecksCurrentLevel_ = (2u * numSparseCollisionChecksPreviousLevel_) + 1u;

            // Restart the reverse search.
            reverseQueue_->clear();
            reverseRoots_.clear();
            reverseCost_ = objective_->infiniteCost();
            for (const auto &goal : graph_.getGoalStates())
            {
                reverseRoots_.emplace_back(goal->asReverseVertex());
            }
            expandReverseRootsIntoReverseQueue();

            // If expanding the goal state actually produced edges, let's start the reverse search.
            // Otherwise, improve the approximation.
            if (!reverseQueue_->empty())
            {
                // Store the edges in the forward queue in the cache.
                jitSearchEdgeCache_ = forwardQueue_->getEdges();
                forwardQueue_->clear();
                phase_ = Phase::REVERSE_SEARCH;
            }
            else
            {
                phase_ = Phase::IMPROVE_APPROXIMATION;
            }

            ++reverseSearchTag_;
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
                auto heuristicPathCost = objective_->combineCosts(
                    objective_->combineCosts(edge.source->getCurrentCostToCome(),
                                             objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw())),
                    objective_->costToGo(edge.target->raw(), problem_->getGoal().get()));
                if (objective_->isCostBetterThan(heuristicPathCost, solutionCost_))
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
                if (!edge.source->hasReverseVertex() || !edge.target->hasReverseVertex())
                {
                    assert(false);
                    return false;
                }
                else
                {
                    return true;
                }
            }
        }

        bool EITstar::couldImproveForwardTree(const Edge &edge) const
        {
            const auto heuristicCostToCome =
                objective_->combineCosts(edge.source->getCurrentCostToCome(),
                                         objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw()));
            return objective_->isCostBetterThan(heuristicCostToCome, edge.target->getCurrentCostToCome());
        }

        bool EITstar::doesImproveForwardPath(const Edge &edge, const ompl::base::Cost &trueEdgeCost) const
        {
            assert(edge.target->asForwardVertex()->getTwin().lock());
            assert(edge.target->hasReverseVertex());
            assert(edge.source->hasForwardVertex());
            if (isAnyReverseRootInForwardTree())
            {
                return objective_->isCostBetterThan(
                    objective_->combineCosts(
                        edge.source->getCurrentCostToCome(),
                        objective_->combineCosts(trueEdgeCost,
                                                 objective_->costToGo(edge.target->raw(), problem_->getGoal().get()))),
                    solutionCost_);
            }
            else
            {
                return true;
            }
        }

        bool EITstar::doesImproveForwardTree(const Edge &edge, const ompl::base::Cost &trueEdgeCost) const
        {
            return objective_->isCostBetterThan(
                objective_->combineCosts(edge.source->getCurrentCostToCome(), trueEdgeCost),
                edge.target->getCurrentCostToCome());
        }

        bool EITstar::isValid(const Edge &edge) const
        {
            // Check if the edge is whitelisted.
            if (edge.source->isWhitelisted(edge.target))
            {
                return true;
            }
            // Check if the edge is blacklisted.
            else if (edge.source->isBlacklisted(edge.target))
            {
                return false;
            }
            // Ok, fine, we have to do work.
            else
            {
                ++numCollisionCheckedEdges_;
                if (motionValidator_->checkMotion(edge.source->raw(), edge.target->raw()))
                {
                    // Whitelist this edge.
                    edge.source->whitelist(edge.target);
                    edge.target->whitelist(edge.source);
                    return true;
                }
                else
                {
                    // Blacklist this edge.
                    edge.source->blacklist(edge.target);
                    edge.target->blacklist(edge.source);

                    // Register it with the graph.
                    graph_.registerInvalidEdge(edge);
                    return false;
                }
            }
        }

        bool EITstar::couldBeValid(const Edge &edge) const
        {
            // Check if the edge is whitelisted.
            if (edge.source->isWhitelisted(edge.target))
            {
                return true;
            }
            // Check if the edge is blacklisted.
            else if (edge.source->isBlacklisted(edge.target))
            {
                return false;
            }
            // Ok, fine, we have to do some work.
            else
            {
                /***
                   Let's say we want to perform seven collision checks on an edge:

                   position of checks:   |--------x--------x--------x--------x--------x--------x--------x--------|
                   indices of checks:             1        2        3        4        5        6        7
                   order of testing:              4        2        5        1        6        3        7

                   We create a queue that holds segments and always test the midpoint of the segments. We start with
                the outermost indices and then break the segment in half until the segment collapses to a single
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
                   7  indices = { (7, 7) }
                      current (7, 7) -> test midpoint = 7 -> add nothing to the queue

                ***/

                // The segment count is the number of checks on this level plus 1.
                auto segmentCount = numSparseCollisionChecksCurrentLevel_ + 1u;

                // If the segment cound is larger than what would be necessary we just do the full collision check
                // which will whitelist the edge.
                if (segmentCount >= space_->validSegmentCount(edge.source->raw(), edge.target->raw()))
                {
                    return isValid(edge);
                }

                // Initialize the queue of positions to be tested.
                std::queue<std::pair<std::size_t, std::size_t>> indices;
                indices.emplace(1u, numSparseCollisionChecksCurrentLevel_);

                // Store the current check number.
                std::size_t currentCheck = 1u;

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
                    if (currentCheck > numSparseCollisionChecksPreviousLevel_)
                    {
                        space_->interpolate(edge.source->raw(), edge.target->raw(),
                                            static_cast<double>(mid) / static_cast<double>(segmentCount),
                                            detectionState_);

                        if (!spaceInfo_->isValid(detectionState_))
                        {
                            edge.source->blacklist(edge.target);
                            edge.target->blacklist(edge.source);
                            return false;
                        }
                    }

                    // Increase the current check number.
                    ++currentCheck;
                }

                return true;
            }
        }

        void EITstar::expandReverseRootsIntoReverseQueue()
        {
            for (auto &reverseRoot : reverseRoots_)
            {
                reverseRoot->getState()->setAdmissibleCostToGo(objective_->identityCost());
                reverseQueue_->insert(expand(reverseRoot->getState()));
            }
        }

        bool EITstar::isAnyForwardRootInReverseTree() const
        {
            for (const auto &root : forwardRoots_)
            {
                if (root->getTwin().lock())
                {
                    return true;
                }
            }
            return false;
        }

        bool EITstar::isAnyReverseRootInForwardTree() const
        {
            for (const auto &root : reverseRoots_)
            {
                if (root->getTwin().lock())
                {
                    return true;
                }
            }
            return false;
        }

        bool EITstar::canBeInsertedInForwardQueue(const eitstar::Edge &edge) const
        {
            return doAllVerticesHaveAdmissibleCostToGo(edge);
        }

        bool EITstar::canBeInsertedInForwardQueue(const std::vector<eitstar::Edge> &edges) const
        {
            return doAllVerticesHaveAdmissibleCostToGo(edges);
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
                    const auto neighborCost =
                        objective_->combineCosts(neighbor->getAdmissibleCostToGo(), neighborEdgeCost);
                    if (objective_->isCostBetterThan(neighborCost, bestCost))
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

        bool EITstar::doAllVerticesHaveAdmissibleCostToGo(const eitstar::Edge &edge) const
        {
            if (!edge.source->hasReverseVertex() || !edge.target->hasReverseVertex())
            {
                return false;
            }

            return isClosed(edge.source->asReverseVertex()) && isClosed(edge.target->asReverseVertex());
        }

        bool EITstar::doAllVerticesHaveAdmissibleCostToGo(const std::vector<eitstar::Edge> &edges) const
        {
            for (const auto &edge : edges)
            {
                if (!doAllVerticesHaveAdmissibleCostToGo(edge))
                {
                    return false;
                }
            }
            return true;
        }

        bool EITstar::doesImproveReversePath(const Edge &edge) const
        {
            // Only do work if there is currently no reverse path.
            if (objective_->isFinite(reverseCost_))
            {
                // Compare the costs of the full path heuristic with the current cost of the start state.
                const auto heuristicPathCost = objective_->combineCosts(
                    objective_->combineCosts(edge.source->getAdmissibleCostToGo(),
                                             objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw())),
                    edge.target->getLowerBoundCostToCome());

                if (objective_->isCostBetterThan(heuristicPathCost, reverseCost_))
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

        bool EITstar::doesImproveReverseTree(const Edge &edge, const ompl::base::Cost &admissibleEdgeCost) const
        {
            return objective_->isCostBetterThan(
                objective_->combineCosts(edge.source->getAdmissibleCostToGo(), admissibleEdgeCost),
                edge.target->getAdmissibleCostToGo());
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
            for (const auto root : forwardRoots_)
            {
                if (root->getTwin().lock())
                {
                    auto edges = expand(root->getState());
                    allEdges.insert(allEdges.end(), edges.begin(), edges.end());
                }
            }
            return allEdges;
        }

        void EITstar::updateJitSearchEdgeCache()
        {
            const auto insertionPredicate = [this](const auto &edge) {
                if (canBeInsertedInForwardQueue(edge))
                {
                    forwardQueue_->insert(edge);
                    return true;
                }
                else
                {
                    return false;
                }
            };

            jitSearchEdgeCache_.erase(
                std::remove_if(jitSearchEdgeCache_.begin(), jitSearchEdgeCache_.end(), insertionPredicate),
                jitSearchEdgeCache_.end());
        }

    }  // namespace geometric
}  // namespace ompl

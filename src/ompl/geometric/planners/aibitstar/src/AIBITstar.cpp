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

#include <dbg.h>

#include "ompl/geometric/planners/aibitstar/AIBITstar.h"

#include <algorithm>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/planners/aibitstar/stopwatch/timetable.h"

using namespace std::string_literals;

namespace ompl
{
    namespace geometric
    {
        using namespace aibitstar;

        AIBITstar::AIBITstar(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo)
          : ompl::base::Planner(spaceInfo, "AI-BIT*")
          , graph_(spaceInfo)
          , space_(spaceInfo->getStateSpace())
          , motionValidator_(spaceInfo->getMotionValidator())
        {
        }

        void AIBITstar::setup()
        {
            // Check that the problem definition is set.
            if (!problem_)
            {
                OMPL_ERROR("AI-BIT* can not be setup without first setting the problem definition.");
                return;
            }

            // Check the goal is of appropriate type.
            if (!problem_->getGoal()->hasType(ompl::base::GOAL_STATE))
            {
                OMPL_ERROR("AI-BIT* currently only works for single goal states.");
                return;
            }

            // Call the base class setup.
            Planner::setup();

            // Default to path length optimization if no objective has been specified.
            if (!problem_->hasOptimizationObjective())
            {
                OMPL_WARN("%s: No optimization objective has been specified. The default is optimizing path length.",
                          name_.c_str());
                problem_->setOptimizationObjective(
                    std::make_shared<ompl::base::PathLengthOptimizationObjective>(spaceInfo_));
            }

            // Let the graph know about the problem so that it can focus its approximation.
            graph_.setProblemDefinition(problem_);

            // Pull through the optimization objective for direct access.
            objective_ = problem_->getOptimizationObjective();

            // Set the best cost to infinity.
            bestCost_ = objective_->infiniteCost();

            // Instantiate the queues.
            forwardQueue_ = std::make_unique<aibitstar::ForwardQueue>(objective_, spaceInfo_);
            reverseQueue_ = std::make_unique<aibitstar::ReverseQueue>(objective_);

            // Add the start state.
            while (Planner::pis_.haveMoreStartStates())
            {
                if (const auto start = Planner::pis_.nextStart())
                {
                    auto startState = graph_.setStartState(start);
                    assert(graph_.hasStartState());
                    forwardRoot_ = startState->asForwardVertex();
                    forwardRoot_->setCost(objective_->identityCost());
                }
            }

            // Add the goal state.
            while (Planner::pis_.haveMoreGoalStates())
            {
                if (const auto goal = Planner::pis_.nextGoal())
                {
                    auto goalState = graph_.setGoalState(goal);
                    assert(graph_.hasGoalState());
                    reverseRoot_ = goalState->asReverseVertex();
                    reverseRoot_->setCost(objective_->identityCost());
                    reverseRoot_->getState()->setEstimatedEffortToGo(0u);
                }
            }
        }

        ompl::base::PlannerStatus AIBITstar::solve(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // Make sure everything is setup.
            if (!setup_)
            {
                throw std::runtime_error("Called solve on AI-BIT* without setting up the planner first.");
            }
            if (!spaceInfo_->isSetup())
            {
                throw std::runtime_error("Called solve on AI-BIT* without setting up the state space first.");
            }

            // If this is the first time solve is being called, populate the reverse queue.
            if (iteration_ == 0u)
            {
                reverseQueue_->insert(expand(reverseRoot_->getState()));
                reverseRoot_->setExtendedCost(objective_->identityCost());
                reverseRoot_->setExpandTag(searchTag_);
            }

            // Iterate until stopped.
            while (!terminationCondition)
            {
                iterate();
            }

            // Return the appropriate planner status.
            if (reverseRoot_->getTwin().lock())
            {
                return ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION;
            }
            else
            {
                return ompl::base::PlannerStatus::StatusType::TIMEOUT;
            }
        }

        ompl::base::Cost AIBITstar::bestCost() const
        {
            return bestCost_;
        }

        void AIBITstar::setNumSamplesPerBatch(std::size_t numSamples)
        {
            numSamplesPerBatch_ = numSamples;
        }

        void AIBITstar::setRadiusFactor(double factor)
        {
            graph_.setRadiusFactor(factor);
        }

        std::vector<Edge> AIBITstar::getForwardQueue() const
        {
            return forwardQueue_->getEdges();
        }

        std::vector<Edge> AIBITstar::getReverseQueue() const
        {
            return reverseQueue_->getEdges();
        }

        std::vector<Edge> AIBITstar::getReverseTree() const
        {
            // Prepare the return value.
            std::vector<Edge> edges;

            // Get the edges recursively.
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
            getEdgesRecursively(reverseRoot_);

            // Return all edges in the reverse tree.
            return edges;
        }

        Edge AIBITstar::getNextForwardEdge() const
        {
            return forwardQueue_->peek(suboptimalityFactor_);
        }

        Edge AIBITstar::getNextReverseEdge() const
        {
            return reverseQueue_->peek();
        }

        void AIBITstar::getPlannerData(base::PlannerData &data) const
        {
            // Get the base class data.
            Planner::getPlannerData(data);

            // Add the samples and their outgoing edges.
            for (const auto &sample : graph_.getSamples())
            {
                // Add the state as a vertex.
                data.addVertex(base::PlannerDataVertex(sample->raw(), sample->getId()));

                // If the sample is in the forward tree, add the outgoing edges.
                if (sample->hasForwardVertex())
                {
                    for (const auto &child : sample->asForwardVertex()->getChildren())
                    {
                        data.addEdge(base::PlannerDataVertex(sample->asForwardVertex()->getState()->raw(),
                                                             sample->asForwardVertex()->getId()),
                                     base::PlannerDataVertex(child->getState()->raw(), child->getId()));
                    }
                }
            }
        }

        void AIBITstar::iterate()
        {
            // Increment the iteration count.
            ++iteration_;

            switch (phase_)
            {
                case Phase::REVERSE_SEARCH:
                {
                    reverseIterate();
                    break;
                }
                case Phase::FORWARD_SEARCH:
                {
                    forwardIterate();
                    break;
                }
                case Phase::IMPROVE_APPROXIMATION:
                {
                    // Add new states.
                    graph_.addStates(numSamplesPerBatch_);

                    // Reset the suboptimality factor.
                    suboptimalityFactor_ = std::numeric_limits<float>::infinity();

                    // Restart the reverse search.
                    reverseRoot_.reset();
                    reverseRoot_ = graph_.getGoalState()->asReverseVertex();
                    reverseRoot_->setCost(objective_->identityCost());
                    reverseRoot_->setExtendedCost(objective_->identityCost());
                    reverseRoot_->setExpandTag(searchTag_);
                    reverseRoot_->getState()->setEstimatedEffortToGo(0u);
                    reverseQueue_->insert(expand(reverseRoot_->getState()));

                    // If expanding the goal state actually produced edges, let's start the reverse search.
                    // Otherwise, we stay in the improve approximation phase.
                    if (!reverseQueue_->empty())
                    {
                        phase_ = Phase::REVERSE_SEARCH;
                    }
                    break;
                }
                default:
                {
                    // We should never reach this.
                    assert(false);
                }
            };
        }

        void AIBITstar::forwardIterate()
        {
            // Ensure the forward queue is not empty.
            assert(!forwardQueue_->empty());

            // Get the top edge from the queue.
            auto edge = forwardQueue_->pop(suboptimalityFactor_);

            // Assert that the source of the edge has a forward vertex.
            assert(edge.source->hasForwardVertex());
            auto parentVertex = edge.source->asForwardVertex();
            auto childVertex = edge.target->asForwardVertex();

            // The forward search is done if this edge can not possibly improve the forward path.
            if (couldImproveForwardPath(edge))
            {
                // Check if the edge's parent is already the parent of the child.
                if (auto currentParent = childVertex->getParent().lock())
                {
                    if (currentParent->getId() == parentVertex->getId() &&
                        edge.target->getId() != graph_.getGoalState()->getId())
                    {
                        forwardQueue_->insert(expand(edge.target));
                        return;
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

                        // Check if the edge can actually improve the forward path and tree.
                        if (doesImproveForwardPath(edge, trueEdgeCost) && doesImproveForwardTree(edge, trueEdgeCost))
                        {
                            // Update the parent of the child in the forward tree.
                            childVertex->updateParent(parentVertex);

                            // Set the edge cost associated with this parent.
                            childVertex->setEdgeCost(trueEdgeCost);

                            // Update the cost-to-come.
                            childVertex->updateCost(objective_);

                            // Update the cost of the children.
                            auto changedVertices = childVertex->updateChildren(objective_);

                            // Update the edges in the queue.
                            for (const auto &vertex : changedVertices)
                            {
                                forwardQueue_->update({vertex->getParent().lock()->getState(), vertex->getState()});

                                if (vertex->getState()->getId() == reverseRoot_->getState()->getId())
                                {
                                    updateSolution();
                                }
                            }

                            // Add the child to the parents children.
                            parentVertex->addChild(childVertex);

                            // Expand the outgoing edges into the queue unless this state is the goal state.
                            if (edge.target->getId() != graph_.getGoalState()->getId())
                            {
                                // Expand the child vertex.
                                forwardQueue_->insert(expand(edge.target));
                            }
                            else  // It is the goal state, update the solution.
                            {
                                updateSolution();
                            }
                        }
                    }
                    else
                    {
                        // Assert the edge is actually invalid.
                        assert(!motionValidator_->checkMotion(edge.source->raw(), edge.target->raw()));

                        dbg("Edge ("s + std::to_string(edge.source->getId()) + " -> "s +
                            std::to_string(edge.target->getId()) + ") is invalid."s);

                        // If this edge was in the reverse search tree, the tree must be updated.
                        if (edge.source->asReverseVertex()->getParent().lock()->getId() ==
                            edge.target->asReverseVertex()->getId())
                        {
                            // Get the source state as reverse vectex.
                            auto reverseSource = edge.source->asReverseVertex();

                            // The edge is invalid. The reverse search can be updated.
                            reverseSource->setEdgeCost(objective_->infiniteCost());
                            reverseSource->setCost(objective_->infiniteCost());

                            // The source state has been invalidated in the reverse search tree. Get the neighbors of
                            // the source state. Find the best new parent in the reverse search.
                            std::shared_ptr<aibitstar::State> newParent;
                            ompl::base::Cost newCost = objective_->infiniteCost();
                            ompl::base::Cost newEdgeCost = objective_->infiniteCost();
                            for (const auto &neighbor : graph_.getNeighbors(edge.source))
                            {
                                dbg(neighbor->getId());
                                auto neighborEdgeCost =
                                    objective_->motionCostBestEstimate(neighbor->raw(), edge.source->raw());
                                auto neighborCost =
                                    objective_->combineCosts(neighbor->asReverseVertex()->getCost(), neighborEdgeCost);
                                if (objective_->isCostBetterThan(neighborCost, newCost))
                                {
                                    newParent = neighbor;
                                    newCost = neighborCost;
                                    newEdgeCost = neighborEdgeCost;
                                }
                            }

                            if (dbg(newParent))
                            {
                                // Update the reverse search tree.
                                newParent->asReverseVertex()->addChild(edge.source->asReverseVertex());
                                edge.target->asReverseVertex()->removeChild(edge.source->asReverseVertex());
                                edge.source->asReverseVertex()->updateParent(newParent->asReverseVertex());
                                edge.source->asReverseVertex()->setEdgeCost(newEdgeCost);
                                edge.source->asReverseVertex()->setCost(newCost);
                                dbg("Updating children.");
                                auto updatedChildren = edge.source->asReverseVertex()->updateChildren(objective_);

                                dbg("Done.");

                                // Update the underlying state.
                                edge.source->setEstimatedCostToGo(newCost);
                                edge.source->setEstimatedEffortToGo(newParent->getEstimatedEffortToGo() +
                                                                    spaceInfo_->getStateSpace()->validSegmentCount(
                                                                        newParent->raw(), edge.source->raw()));

                                dbg("Updated the underlying state.");

                                // Update the underlying states of all updated children. We can do this in sequence,
                                // because thats how they are returned in Vertex::updateChildren. Although this
                                // assumption is risky, because the implementation could change. TODO(Marlin): Make this
                                // not dependent on the assumption above.
                                for (const auto &child : updatedChildren)
                                {
                                    assert(child->getParent().lock());
                                    auto parent = child->getParent().lock();
                                    auto parentState = parent->getState();
                                    auto childState = child->getState();
                                    childState->setEstimatedCostToGo(child->getCost());
                                    childState->setEstimatedEffortToGo(parentState->getEstimatedEffortToGo() +
                                                                       spaceInfo_->getStateSpace()->validSegmentCount(
                                                                           parentState->raw(), childState->raw()));
                                }

                                dbg("Updated the states underlying the children.");

                                // Update the position of the outgoing edges of all updated children in the forward
                                // search queue.
                                for (const auto &child : updatedChildren)
                                {
                                    assert(child->getParent().lock());
                                    auto parent = child->getParent().lock();
                                    forwardQueue_->update(Edge(parent->getState(), child->getState()));
                                }

                                dbg("Updated the queue.");
                            }
                        }
                    }
                }
            }
            // If the forward queue is empty, move on to the next phase.
            if (forwardQueue_->empty() ||
                (reverseRoot_->getTwin().lock() &&
                 bestCost_.value() / forwardQueue_->getLowerBoundOnOptimalSolutionCost().value() < 2.0))
            {
                forwardQueue_->clear();
                phase_ = Phase::IMPROVE_APPROXIMATION;
            }
        }

        void AIBITstar::reverseIterate()
        {
            // Ensure the reverse queue is not empty.
            assert(!reverseQueue_->empty());

            // Get the top edge from the queue.
            auto edge = reverseQueue_->pop();

            // The parent vertex must have an associated vertex in the tree.
            assert(edge.source->hasReverseVertex());

            // Get the parent and child vertices.
            auto parentVertex = edge.source->asReverseVertex();
            auto childVertex = edge.target->asReverseVertex();

            // Simply expand the child vertex if the edge is already in the reverse tree, and the child has not been
            // expanded yet.
            if (auto currentParent = childVertex->getParent().lock())
            {
                if (!isClosed(childVertex) && currentParent->getId() == edge.source->asReverseVertex()->getId())
                {
                    reverseQueue_->insert(expand(edge.target));
                    childVertex->setExtendedCost(childVertex->getCost());
                    childVertex->setExpandTag(searchTag_);
                    return;
                }
            }

            // Incorporate the edge in the reverse tree if it provides an improvement.
            if (doesImproveReverseTree(edge))
            {
                // Update the parent of the child in the reverse tree.
                childVertex->updateParent(parentVertex);

                // Set the edge cost.
                childVertex->setEdgeCost(objective_->motionCostBestEstimate(parentVertex->getState()->raw(),
                                                                            childVertex->getState()->raw()));

                // Update the cost of the vertex in the tree.
                childVertex->updateCost(objective_);

                // The cost-to-come of the vertex in the tree is our estimate of the cost-to-go of the underlying state.
                edge.target->setEstimatedCostToGo(childVertex->getCost());

                // Add the child to the children of the parent.
                parentVertex->addChild(childVertex);

                // The effort-to-come of the vertex in the tree is our estimate of the effort-to-go of the underlying
                // state.
                edge.target->setEstimatedEffortToGo(
                    edge.source->getEstimatedEffortToGo() +
                    spaceInfo_->getStateSpace()->validSegmentCount(edge.source->raw(), edge.target->raw()));

                // Expand the outgoing edges into the queue unless this has already happened or this is the start state.
                if (!isClosed(childVertex) && edge.target->getId() != forwardRoot_->getState()->getId())
                {
                    reverseQueue_->insert(expand(edge.target));
                    childVertex->setExtendedCost(childVertex->getCost());
                    childVertex->setExpandTag(searchTag_);
                }
            }

            // Check if this was the last edge in the reverse queue.
            // TODO: This means we always check all edges in the graph. Is this really necessary?
            if (reverseQueue_->empty())
            {
                // Update the search tag.
                ++searchTag_;

                // Insert the outgoing edges of the start into the forward queue if there could be a path.
                if (forwardRoot_->getTwin().lock())
                {
                    assert(forwardRoot_->getState()->hasReverseVertex());
                    forwardQueue_->insert(expand(forwardRoot_->getState()));
                    phase_ = Phase::FORWARD_SEARCH;
                }
                else
                {
                    phase_ = Phase::IMPROVE_APPROXIMATION;
                }
            }
        }

        void AIBITstar::updateSolution()
        {
            // Throw if the reverse root does not have a forward vertex.
            assert(reverseRoot_->getTwin().lock());
            assert(reverseRoot_->getState()->hasForwardVertex());
            auto current = reverseRoot_->getTwin().lock();

            // Update the best cost.
            bestCost_ = current->getCost();

            // Allocate the path.
            auto path = std::make_shared<ompl::geometric::PathGeometric>(spaceInfo_);

            // Allocate a vector for vertices. The append function of the path inserts vertices in front of an
            // std::vector, which is not very efficient. I'll rather iterate over the vector in reverse.
            std::vector<std::shared_ptr<Vertex>> vertices;

            // Continuously append vertices.
            while (current->getState()->getId() != forwardRoot_->getState()->getId())
            {
                assert(current->getParent().lock());
                vertices.emplace_back(current);
                current = current->getParent().lock();
            }
            vertices.emplace_back(current);

            // Append all vertices to the path.
            for (auto it = vertices.rbegin(); it != vertices.rend(); ++it)
            {
                assert((*it)->getState());
                assert((*it)->getState()->raw());
                path->append((*it)->getState()->raw());
            }

            // Register this solution with the problem definition.
            ompl::base::PlannerSolution solution(path);
            solution.setPlannerName(name_);
            solution.setOptimized(objective_, bestCost_, objective_->isSatisfied(bestCost_));
            problem_->addSolutionPath(solution);

            // Set a new suboptimality factor.
            suboptimalityFactor_ = bestCost_.value() / forwardQueue_->getLowerBoundOnOptimalSolutionCost().value();
        }

        bool AIBITstar::couldImproveForwardPath(const Edge &edge) const
        {
            // If the start state has not been discovered by the reverse search, the answer is always yes.
            if (auto goalVertex = reverseRoot_->getTwin().lock())
            {
                assert(reverseRoot_->getState()->hasForwardVertex());
                // Compare the costs of the full path heuristic with the current cost of the start state.
                auto heuristicPathCost = objective_->combineCosts(
                    objective_->combineCosts(edge.source->asForwardVertex()->getCost(),
                                             objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw())),
                    objective_->costToGo(edge.target->raw(), problem_->getGoal().get()));
                if (objective_->isCostBetterThan(heuristicPathCost, goalVertex->getCost()))
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
                    return false;
                }
                else
                {
                    return true;
                }
            }
        }

        bool AIBITstar::couldImproveForwardTree(const Edge &edge) const
        {
            auto heuristicCostToCome =
                objective_->combineCosts(edge.source->asForwardVertex()->getCost(),
                                         objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw()));
            return objective_->isCostBetterThan(heuristicCostToCome, edge.target->asForwardVertex()->getCost());
        }

        bool AIBITstar::doesImproveForwardPath(const Edge &edge, const ompl::base::Cost &trueEdgeCost) const
        {
            assert(edge.target->asForwardVertex()->getTwin().lock());
            assert(edge.target->hasReverseVertex());
            assert(edge.source->hasForwardVertex());
            if (auto reverseRootForwardVertex = reverseRoot_->getTwin().lock())
            {
                assert(reverseRoot_->getState()->hasForwardVertex());
                return objective_->isCostBetterThan(
                    objective_->combineCosts(
                        edge.source->asForwardVertex()->getCost(),
                        objective_->combineCosts(trueEdgeCost,
                                                 edge.target->asForwardVertex()->getTwin().lock()->getCost())),
                    reverseRootForwardVertex->getCost());
            }
            else
            {
                assert(!reverseRoot_->getState()->hasForwardVertex());
                return true;
            }
        }

        bool AIBITstar::doesImproveForwardTree(const Edge &edge, const ompl::base::Cost &trueEdgeCost) const
        {
            return objective_->isCostBetterThan(
                objective_->combineCosts(edge.source->asForwardVertex()->getCost(), trueEdgeCost),
                edge.target->asForwardVertex()->getCost());
        }

        bool AIBITstar::isValid(const Edge &edge) const
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
            else if (motionValidator_->checkMotion(edge.source->raw(), edge.target->raw()))
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

        void AIBITstar::rebuildForwardQueue()
        {
            // This is going to be a bit messy. Get the edges in the queue.
            auto edges = forwardQueue_->getEdges();

            // All edges have an updated key, lets rebuild the queue.
            forwardQueue_->clear();
            forwardQueue_->insert(edges);
        }

        bool AIBITstar::isClosed(const std::shared_ptr<Vertex> &vertex) const
        {
            return vertex->getExpandTag() == searchTag_;
        }

        bool AIBITstar::doesImproveReversePath(const Edge &edge) const
        {
            // If the start state has not been discovered by the reverse search, the answer is always yes.
            if (auto startVertex = forwardRoot_->getTwin().lock())
            {
                assert(forwardRoot_->getState()->hasReverseVertex());
                // Compare the costs of the full path heuristic with the current cost of the start state.
                auto heuristicPathCost = objective_->combineCosts(
                    objective_->combineCosts(edge.source->asReverseVertex()->getCost(),
                                             objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw())),
                    objective_->motionCostHeuristic(edge.target->raw(), forwardRoot_->getState()->raw()));
                if (objective_->isCostBetterThan(heuristicPathCost, startVertex->getCost()))
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

        bool AIBITstar::doesImproveReverseTree(const Edge &edge) const
        {
            return objective_->isCostBetterThan(
                objective_->combineCosts(edge.source->asReverseVertex()->getCost(),
                                         objective_->motionCostBestEstimate(edge.source->raw(), edge.target->raw())),
                edge.target->asReverseVertex()->getCost());
        }

        std::vector<Edge> AIBITstar::expand(const std::shared_ptr<State> &state) const
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
                if (forwardVertex->getId() != forwardRoot_->getId())
                {
                    // If this vertex is not the forward root, it must have a parent.
                    assert(forwardVertex->getParent().lock());

                    // Get the state associated with the parent vertex.
                    auto forwardParentState = forwardVertex->getParent().lock()->getState();

                    // Add the edge to the forward tree parent.
                    outgoingEdges.emplace_back(state, forwardParentState);
                }

                // Add the edge to the forward children.
                for (const auto &child : forwardVertex->getChildren())
                {
                    // Get the state associated with the child vertex.
                    auto forwardChildState = child->getState();

                    // Add the edge to the child to the outgoing edges.
                    outgoingEdges.emplace_back(state, forwardChildState);
                }
            }

            // If the state is in the reverse search tree, extra edges have to be added.
            if (state->hasReverseVertex())
            {
                // Get the vertex in the reverse search tree associated with this state.
                auto reverseVertex = state->asReverseVertex();

                // Add the outgoing edge to this vertex's parent in the reverse tree, if it exists.
                if (reverseVertex->getId() != reverseRoot_->getId())
                {
                    // If this vertex is not the reverse root, it must have a parent.
                    assert(reverseVertex->getParent().lock());

                    // Get the state associated with the parent vertex.
                    auto reverseParentState = reverseVertex->getParent().lock()->getState();

                    // Add the edge to the reverse tree parent.
                    outgoingEdges.emplace_back(state, reverseParentState);
                }

                // Add the edge to the reverse children.
                for (const auto &child : reverseVertex->getChildren())
                {
                    // Get the state associated with the child vertex.
                    auto reverseChildState = child->getState();

                    // Add the edge to the child to the outgoing edges.
                    outgoingEdges.emplace_back(state, reverseChildState);
                }
            }

            return outgoingEdges;
        }

    }  // namespace geometric
}  // namespace ompl

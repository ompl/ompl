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

#include "ompl/geometric/planners/aibitstar/AIBITstar.h"

#include <algorithm>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/PathGeometric.h"

namespace ompl
{
    namespace geometric
    {
        using namespace aibitstar;

        AIBITstar::AIBITstar(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo)
          : ompl::base::Planner(spaceInfo, "AI-BIT*")
          , graph_(spaceInfo)
          , motionValidator_(spaceInfo->getMotionValidator())
        {
        }

        void AIBITstar::setup()
        {
            // Check that the problem definition is set.
            if (!problem_)
            {
                OMPL_ERROR("AI-BIT* can not be setup without first setting the probelm definition.");
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
                OMPL_WARN("%s: No optimization has been specified. Defaulting to path length.", name_.c_str());
                problem_->setOptimizationObjective(
                    std::make_shared<ompl::base::PathLengthOptimizationObjective>(spaceInfo_));
            }

            // Let the graph know about the problem so that it can focus its approximation.
            graph_.setProblemDefinition(problem_);

            // Pull through the optimization objective for direct access.
            objective_ = problem_->getOptimizationObjective();

            // Add the start state.
            while (Planner::pis_.haveMoreStartStates())
            {
                if (const auto start = Planner::pis_.nextStart())
                {
                    auto startState = graph_.setStartState(start);
                    assert(graph_.hasStartState());
                    forwardRoot_ = std::make_shared<Vertex>(startState);
                    forwardRoot_->setCost(objective_->identityCost());
                    startState->setForwardVertex(forwardRoot_);
                }
            }

            // Add the goal state.
            while (Planner::pis_.haveMoreGoalStates())
            {
                if (const auto goal = Planner::pis_.nextGoal())
                {
                    auto goalState = graph_.setGoalState(goal);
                    assert(graph_.hasGoalState());
                    reverseRoot_ = std::make_shared<Vertex>(goalState);
                    reverseRoot_->setCost(objective_->identityCost());
                    goalState->setReverseVertex(reverseRoot_);
                }
            }
        }

        ompl::base::PlannerStatus AIBITstar::solve(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // Make sure everything is setup.
            if (!setup_)
            {
                throw std::runtime_error("Called solve on AIBIT* without setting up the planner first.");
            }
            if (!spaceInfo_->isSetup())
            {
                throw std::runtime_error("Called solve on AIBIT* without setting up the state space first.");
            }

            // If this is the first time solve is being called, populate the backward queue.
            if (iteration_ == 0u)
            {
                reverseQueue_.insert(reverseExpand(graph_.getGoalState()));
            }

            // Iterate until stopped.
            while (!terminationCondition)
            {
                iterate();
            }

            // Return the appropriate planner status.
            if (reverseRoot_->getState()->getForwardVertex().lock())
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
            if (auto forwardGoal = reverseRoot_->getState()->getForwardVertex().lock())
            {
                return forwardGoal->getCost();
            }
            else
            {
                return objective_->infiniteCost();
            }
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
            return forwardQueue_.getEdges();
        }

        std::vector<Edge> AIBITstar::getReverseQueue() const
        {
            return reverseQueue_.getEdges();
        }

        std::vector<Edge> AIBITstar::getReverseTree() const
        {
            // Prepare the return value.
            std::vector<Edge> edges;

            // Get the edges recursively.
            std::function<void(const std::shared_ptr<Vertex> &)> getEdgesRecursively =
                [this, &edges, &getEdgesRecursively](const std::shared_ptr<Vertex> &vertex) {
                    for (const auto &child : vertex->getChildren())
                    {
                        getEdgesRecursively(child);
                    }
                    // Catch the root case.
                    if (auto parent = vertex->getParent().lock())
                    {
                        edges.emplace_back(createForwardEdge(parent, vertex));
                    }
                };
            getEdgesRecursively(reverseRoot_);

            // Return all edges in the reverse tree.
            return edges;
        }

        Edge AIBITstar::getNextForwardEdge() const
        {
            return forwardQueue_.peek();
        }

        Edge AIBITstar::getNextReverseEdge() const
        {
            return reverseQueue_.peek();
        }

        void AIBITstar::getPlannerData(base::PlannerData &data) const
        {
            // Get the base class data.
            Planner::getPlannerData(data);

            // Add the samples and their outgoing edges.
            for (const auto &sample : graph_.getSamples())
            {
                // Add the state as a vertex.
                data.addVertex(base::PlannerDataVertex(sample->getState(), sample->getId()));

                // If the sample is in the forward tree, add the outgoing edges.
                if (auto forwardVertex = sample->getForwardVertex().lock())
                {
                    for (const auto &child : forwardVertex->getChildren())
                    {
                        data.addEdge(
                            base::PlannerDataVertex(forwardVertex->getState()->getState(), forwardVertex->getId()),
                            base::PlannerDataVertex(child->getState()->getState(), child->getId()));
                    }
                }
            }
        }

        void AIBITstar::iterate()
        {
            // ---------------------------------------
            // Check whether the solution cost is updated.
            static auto solutionCost = objective_->infiniteCost();

            if (auto goalVertex = reverseRoot_->getState()->getForwardVertex().lock())
            {
                if (!objective_->isCostEquivalentTo(solutionCost, goalVertex->getCost()))
                {
                    std::cout << "Goal vertex (" << goalVertex->getId() << ") has updated cost of "
                              << goalVertex->getCost().value() << " through parent "
                              << goalVertex->getParent().lock()->getId() << '\n';
                    solutionCost = goalVertex->getCost();
                }
            }

            // ---------------------------------------
            ++iteration_;
            if (!reverseQueue_.empty())
            {
                reverseIterate();
            }
            else if (!forwardQueue_.empty())
            {
                forwardIterate();
            }
            else
            {
                graph_.addStates(numSamplesPerBatch_);
                reverseQueue_.insert(reverseExpand(graph_.getGoalState()));
            }
        }

        void AIBITstar::forwardIterate()
        {
            // Ensure the forward queue is not empty.
            assert(!forwardQueue_.empty());

            // Get the top edge from the queue.
            auto edge = forwardQueue_.pop();

            // The forward search is done if this edge can not possibly improve the forward path.
            if (canImproveForwardPath(edge))
            {
                // Check if the edge's parent is already the parent of the child.
                if (auto currentParent = edge.child->getParent().lock())
                {
                    if (currentParent->getId() == edge.parent->getId())
                    {
                        if (!isClosed(edge.child))
                        {
                            forwardQueue_.insert(forwardExpand(edge.child->getState()));
                            return;
                        }
                    }
                }
                // Check if it can possibly improve the tree.
                if (canImproveForwardTree(edge))
                {
                    // Check if the edge is valid.
                    if (isValid(edge))
                    {
                        auto trueEdgeCost = objective_->motionCost(edge.parent->getState()->getState(),
                                                                   edge.child->getState()->getState());
                        // Check if the edge can actually improve the forward path and tree.
                        if (doesImproveForwardPath(edge, trueEdgeCost) && doesImproveForwardTree(edge, trueEdgeCost))
                        {
                            // Update the parent of the child in the forward tree.
                            edge.child->setParent(edge.parent);

                            // Update the cost.
                            edge.child->setCost(objective_->combineCosts(edge.parent->getCost(), trueEdgeCost));

                            // Add the child to the parents children.
                            edge.parent->addChild(edge.child);

                            // Expand the outgoing edges into the queue unless this state is the goal state.
                            if (!isClosed(edge.child) &&
                                edge.child->getState()->getId() != reverseRoot_->getState()->getId())
                            {
                                forwardQueue_.insert(forwardExpand(edge.child->getState()));
                            }
                            else  // Update the solution.
                            {
                                updateSolution();
                            }
                        }
                    }
                    else
                    {
                        // The accuracy of the heuristic should be updated.
                        processInvalidEdge(edge);

                        // Restart the reverse queue.
                        reverseQueue_.insert(reverseExpand(reverseRoot_->getState()));
                    }
                }
            }
            else  // The forward search is done.
            {
                if (edge.parent->getId() == 0u && edge.child->getId() == 267)
                {
                    std::cout << "[F] Can not improve solution";
                }
                forwardQueue_.clear();
                ++searchTag_;
            }
        }

        void AIBITstar::updateSolution() const
        {
            // Throw if the reverse root does not have a forward vertex.
            assert(reverseRoot_->getState()->getForwardVertex().lock());
            auto current = reverseRoot_->getState()->getForwardVertex().lock();

            // Allocate the path.
            auto path = std::make_shared<ompl::geometric::PathGeometric>(spaceInfo_);

            // Allocate a vector for vertices. The append function of the path inserts vertices in front of an
            // std::vector, which is not what I want at all. I'll rather call std::reverse in the end.
            std::vector<std::shared_ptr<Vertex>> vertices;

            // Continuously append vertices.
            while (current->getState()->getId() != forwardRoot_->getState()->getId())
            {
                vertices.emplace_back(current);
                assert(current->getParent().lock());
                current = current->getParent().lock();
            }
            vertices.emplace_back(current);

            // Append all vertices to the path.
            for (auto it = vertices.rbegin(); it != vertices.rend(); ++it)
            {
                path->append((*it)->getState()->getState());
            }

            // Register this solution with the problem definition.
            ompl::base::PlannerSolution solution(path);
            solution.setPlannerName(name_);
            solution.optimized_ =
                objective_->isSatisfied(reverseRoot_->getState()->getForwardVertex().lock()->getCost());
            problem_->addSolutionPath(solution);
        }

        void AIBITstar::reverseIterate()
        {
            // Ensure the reverse queue is not empty.
            assert(!reverseQueue_.empty());

            // Get the top edge from the queue.
            auto edge = reverseQueue_.pop();

            // The reverse search is done if this edge can not improve the reverse path.
            if (canImproveReversePath(edge))
            {
                if (auto currentParent = edge.child->getParent().lock())
                {
                    if (!isClosed(edge.child) && currentParent->getId() == edge.parent->getId())
                    {
                        reverseQueue_.insert(reverseExpand(edge.child->getState()));
                        return;
                    }
                }
                if (canImproveReverseTree(edge))
                {
                    // Update the parent of the child in the reverse tree.
                    edge.child->setParent(edge.parent);

                    // Update the cost.
                    edge.child->setCost(ompl::base::Cost(edge.key[1]));

                    // Add the child to the children of the parent.
                    edge.parent->addChild(edge.child);

                    // Expand the outgoing edges into the queue unless this is the start state.
                    if (!isClosed(edge.child) && edge.child->getState()->getId() != forwardRoot_->getState()->getId())
                    {
                        reverseQueue_.insert(reverseExpand(edge.child->getState()));
                    }
                }
            }
            else
            {
                // Clear the reverse queue.
                reverseQueue_.clear();

                // Update the search tag.
                ++searchTag_;

                // If there could be a path, rebuild the queue or insert the outgoing edges of the start.
                if (forwardRoot_->getState()->getReverseVertex().lock())
                {
                    // If the queue is empty, insert the outgoing edges of the start.
                    if (forwardQueue_.empty())
                    {
                        forwardQueue_.insert(forwardExpand(forwardRoot_->getState()));
                    }
                    else  // Otherwise rebuild the queue.
                    {
                        rebuildForwardQueue();
                    }
                }
                else  // If the start is not reached by the backward search, there is no need to continue the forward
                      // search.
                {
                    forwardQueue_.clear();
                }
            }
        }

        bool AIBITstar::canImproveForwardPath(const Edge &edge) const
        {
            if (edge.parent->getId() == 0u && edge.child->getId() == 273)
            {
                std::cout << "[F] Checking whether edge (0 -> 273) can improve solution.\n";
                if (auto goalVertex = reverseRoot_->getState()->getForwardVertex().lock())
                {
                    std::cout << "Reverse root is connected in the forward tree.\n";
                    std::cout << "Key[0]: " << edge.key[0] << ", solution cost: " << goalVertex->getCost().value()
                              << '\n';
                    std::cout << "Is Key[0] better: " << std::boolalpha
                              << objective_->isCostBetterThan(ompl::base::Cost(edge.key[0]), goalVertex->getCost())
                              << '\n';
                }
                else
                {
                    std::cout << "Reverse root is not connected in the forward tree.\n";
                }
            }

            // If the start state has not been discovered by the reverse search, the answer is always yes.
            if (auto goalVertex = reverseRoot_->getState()->getForwardVertex().lock())
            {
                // Compare the costs of the full path heuristic with the current cost of the start state.
                if (objective_->isCostBetterThan(ompl::base::Cost(edge.key[0]), goalVertex->getCost()))
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

        bool AIBITstar::canImproveForwardTree(const Edge &edge) const
        {
            return objective_->isCostBetterThan(ompl::base::Cost(edge.key[1]), edge.child->getCost());
        }

        bool AIBITstar::doesImproveForwardPath(const Edge &edge, const ompl::base::Cost &trueEdgeCost) const
        {
            assert(edge.child->getState()->getReverseVertex().lock());
            if (auto reverseRootForwardVertex = reverseRoot_->getState()->getForwardVertex().lock())
            {
                return objective_->isCostBetterThan(
                    objective_->combineCosts(
                        edge.parent->getCost(),
                        objective_->combineCosts(trueEdgeCost,
                                                 edge.child->getState()->getReverseVertex().lock()->getCost())),
                    reverseRootForwardVertex->getCost());
            }
            else
            {
                return true;
            }
        }

        bool AIBITstar::doesImproveForwardTree(const Edge &edge, const ompl::base::Cost &trueEdgeCost) const
        {
            return objective_->isCostBetterThan(objective_->combineCosts(edge.parent->getCost(), trueEdgeCost),
                                                edge.child->getCost());
        }

        bool AIBITstar::isValid(const Edge &edge) const
        {
            // Check if the edge is whitelisted.
            if (edge.parent->getState()->isWhitelisted(edge.child->getState()))
            {
                return true;
            }
            // Check if the edge is blacklisted.
            else if (edge.parent->getState()->isBlacklisted(edge.child->getState()))
            {
                return false;
            }
            // Ok, fine, we have to do work.
            else if (motionValidator_->checkMotion(edge.parent->getState()->getState(),
                                                   edge.child->getState()->getState()))
            {
                // Whitelist this edge.
                edge.parent->getState()->whitelist(edge.child->getState());
                edge.child->getState()->whitelist(edge.parent->getState());
                return true;
            }
            else
            {
                // Blacklist this edge.
                edge.parent->getState()->blacklist(edge.child->getState());
                edge.child->getState()->blacklist(edge.parent->getState());
                return false;
            }
        }

        void AIBITstar::processInvalidEdge(const Edge &edge)
        {
            // Assert the edge is actually invalid.
            assert(!isValid(edge));
            assert(edge.parent->getState()->getReverseVertex().lock());
            assert(edge.child->getState()->getReverseVertex().lock());

            // Invalidate the branch.
            auto invalidatedReverseVertex = edge.parent->getState()->getReverseVertex().lock();
            invalidatedReverseVertex->releaseBranchFromStates();
            edge.child->getState()->getReverseVertex().lock()->removeChild(invalidatedReverseVertex);
            edge.child->getState()->resetForwardVertex();

            // Clear the forward queue.
            forwardQueue_.clear();

            // Register the invalid edge with the graph.
            graph_.registerInvalidEdge(edge);

            // Assert we did not invalidate the roots.
            assert(forwardRoot_->getState()->getForwardVertex().lock());
            assert(reverseRoot_->getState()->getReverseVertex().lock());
        }

        void AIBITstar::rebuildForwardQueue()
        {
            // This is going to be a bit messy. Get the edges in the queue.
            auto edges = forwardQueue_.getEdges();

            // Get the parent vertices of the edges.
            std::set<std::shared_ptr<Vertex>> parents;
            for (const auto &edge : edges)
            {
                parents.insert(edge.parent);
            }

            // Update the sort key of all outgoing edges of all parents.
            for (const auto &parent : parents)
            {
                for (const auto &outEdgePointer : parent->getOutQueueLookup())
                {
                    outEdgePointer->data.key = computeForwardKey(
                        outEdgePointer->data.parent, outEdgePointer->data.child, outEdgePointer->data.heuristicCost);
                }
            }

            // All edges have an updated key, lets rebuild the queue.
            forwardQueue_.rebuild();
        }

        bool AIBITstar::isClosed(const std::shared_ptr<Vertex> &vertex) const
        {
            return vertex->getExpandTag() == searchTag_;
        }

        bool AIBITstar::canImproveReversePath(const Edge &edge) const
        {
            // If the start state has not been discovered by the reverse search, the answer is always yes.
            if (auto startVertex = forwardRoot_->getState()->getReverseVertex().lock())
            {
                // Compare the costs of the full path heuristic with the current cost of the start state.
                if (objective_->isCostBetterThan(ompl::base::Cost(edge.key[0]), startVertex->getCost()))
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

        bool AIBITstar::canImproveReverseTree(const Edge &edge) const
        {
            return objective_->isCostBetterThan(objective_->combineCosts(edge.parent->getCost(), edge.heuristicCost),
                                                edge.child->getCost());
        }

        std::vector<Edge> AIBITstar::forwardExpand(const std::shared_ptr<State> &state) const
        {
            // Expanding a state into the forward queue means that the state must have a parent in the forward tree.
            assert(state->getForwardVertex().lock());

            // Prepare the return variable.
            std::vector<Edge> outgoingEdges;

            // Get the forward vertex.
            auto forwardVertex = state->getForwardVertex().lock();

            // Add the outgoing edges to the neighbors in the current graph.
            for (const auto &neighborState : graph_.getNeighbors(state))
            {
                // Get the forward vertex associated with this neighbor.
                auto neighborVertex = neighborState->getForwardVertex().lock();

                // If this state does not have an associated forward vertex, create one.
                if (!neighborVertex)
                {
                    neighborVertex = std::make_shared<Vertex>(neighborState);
                    neighborState->setForwardVertex(neighborVertex);
                }

                // Add the edge to this neighbor to the outgoing edges.
                outgoingEdges.emplace_back(createForwardEdge(forwardVertex, neighborVertex));
            }

            // Add the outgoing edge to this vertex's parent in the forward tree, if it exists.
            if (forwardVertex->getId() != forwardRoot_->getId())
            {
                // If this vertex is not the forward root, it must have a parent.
                assert(forwardVertex->getParent().lock());

                // Add the edge to the reverse tree parent. This seems wrong, but if this edge isn't added and this
                // vertex becomes better connected then we could potentially miss updating the current parent.
                outgoingEdges.emplace_back(createForwardEdge(forwardVertex, forwardVertex->getParent().lock()));
            }

            // Add the edges to the forward children.
            for (const auto &child : forwardVertex->getChildren())
            {
                // Add the edge to the child to the outgoing edges.
                outgoingEdges.emplace_back(createForwardEdge(forwardVertex, child));
            }

            // If this state is in the reverse search tree as well, add its reverse parent and children.
            if (auto reverseVertex = state->getReverseVertex().lock())
            {
                // Add the parent.
                if (reverseVertex->getId() != reverseRoot_->getId())
                {
                    if (auto reverseParent = reverseVertex->getParent().lock())
                    {
                        // Get the forward vertex associated with the reverse parent's state.
                        auto parentState = reverseParent->getState();
                        auto parentVertex = parentState->getForwardVertex().lock();

                        // If this state does not have an associated forward vertex, create one.
                        if (!parentVertex)
                        {
                            parentVertex = std::make_shared<Vertex>(parentState);
                            parentState->setForwardVertex(parentVertex);
                        }

                        // Add the edge to the reverse tree parent.
                        outgoingEdges.emplace_back(createForwardEdge(forwardVertex, parentVertex));
                    }
                }

                for (const auto &child : reverseVertex->getChildren())
                {
                    outgoingEdges.emplace_back(createForwardEdge(forwardVertex, child));
                }
            }

            // Remember that this vertex is expanded.
            forwardVertex->setExpandTag(searchTag_);

            return outgoingEdges;
        }

        std::vector<Edge> AIBITstar::reverseExpand(const std::shared_ptr<State> &state) const
        {
            // Expanding a state into the reverse queue means that the state must have a parent in the reverse tree.
            assert(state->getReverseVertex().lock());

            // Prepare the return variable.
            std::vector<Edge> outgoingEdges;

            // Get the reverse vertex.
            auto reverseVertex = state->getReverseVertex().lock();

            // Get the neighbors in the current graph.
            for (const auto &neighborState : graph_.getNeighbors(state))
            {
                // Get the reverse vertex associated with this neighbor.
                auto neighborVertex = neighborState->getReverseVertex().lock();

                // If this state does not have an associated reverse vertex, create one.
                if (!neighborVertex)
                {
                    neighborVertex = std::make_shared<Vertex>(neighborState);
                    neighborState->setReverseVertex(neighborVertex);
                }

                // Add the edge to this neighbor to the outgoing edges.
                outgoingEdges.emplace_back(createReverseEdge(reverseVertex, neighborVertex));
            }

            // Add the outgoing edge to this vertex's parent in the reverse tree, if it exists.
            if (reverseVertex->getId() != reverseRoot_->getId())
            {
                // If this vertex is not the reverse root, it must have a parent.
                assert(reverseVertex->getParent().lock());

                // Add the edge to the reverse tree parent. This seems wrong, but if this edge isn't added and this
                // vertex becomes better connected then we could potentially miss updating the current parent.
                outgoingEdges.emplace_back(createReverseEdge(reverseVertex, reverseVertex->getParent().lock()));
            }

            // Add the edge to the reverse children.
            for (const auto &child : reverseVertex->getChildren())
            {
                // Add the edge to the child to the outgoing edges.
                outgoingEdges.emplace_back(createReverseEdge(reverseVertex, child));
            }

            // If this state is in the forward search tree as well, add its forward parent and children.
            if (auto forwardVertex = state->getForwardVertex().lock())
            {
                // Add the parent if the forward vertex isn't the root of the forward tree.
                if (forwardVertex->getId() != forwardRoot_->getId())
                {
                    // If this vertex is not the forward root, but in the forward tree, it must have a parent.
                    assert(forwardVertex->getParent().lock());

                    // Get the reverse vertex assocaited with the forward parent's state.
                    auto forwardParent = forwardVertex->getParent().lock();
                    auto parentState = forwardParent->getState();
                    auto parentVertex = parentState->getReverseVertex().lock();

                    // If this state does not have an associated reverse vertex, create one.
                    if (!parentVertex)
                    {
                        parentVertex = std::make_shared<Vertex>(parentState);
                        parentState->setReverseVertex(parentVertex);
                    }

                    // Add the edge to the forward tree parent.
                    outgoingEdges.emplace_back(createReverseEdge(reverseVertex, parentVertex));
                }

                // Add the children.
                for (const auto &child : forwardVertex->getChildren())
                {
                    // Add the edge to the child to the outgoing edges.
                    outgoingEdges.emplace_back(createReverseEdge(reverseVertex, child));
                }
            }

            // Remember that this vertex is expanded.
            reverseVertex->setExpandTag(searchTag_);

            return outgoingEdges;
        }

        aibitstar::Edge AIBITstar::createForwardEdge(const std::shared_ptr<aibitstar::Vertex> &parent,
                                                     const std::shared_ptr<aibitstar::Vertex> &child) const
        {
            // Compute the heuristic cost of the edge.
            auto edgeCost =
                objective_->motionCostHeuristic(parent->getState()->getState(), child->getState()->getState());

            // Return an edge.
            return aibitstar::Edge(parent, child, edgeCost, computeForwardKey(parent, child, edgeCost));
        }

        aibitstar::Edge AIBITstar::createReverseEdge(const std::shared_ptr<aibitstar::Vertex> &parent,
                                                     const std::shared_ptr<aibitstar::Vertex> &child) const
        {
            // Compute the heuristic cost of the edge.
            auto edgeCost =
                objective_->motionCostHeuristic(parent->getState()->getState(), child->getState()->getState());

            // Return an edge.
            return aibitstar::Edge(parent, child, edgeCost, computeReverseKey(parent, child, edgeCost));
        }

        std::array<double, 3u> AIBITstar::computeForwardKey(const std::shared_ptr<Vertex> &parent,
                                                            const std::shared_ptr<Vertex> &child,
                                                            const ompl::base::Cost &edgeCost) const
        {
            // Assert the sanity of the input.
            assert(parent);
            assert(child);

            // The cost to come is the cost through the tree.
            auto costToCome = parent->getCost();

            // The cost to go is infinity, if not in the reverse search tree and the corresponding cost otherwise.
            auto costToGo = objective_->infiniteCost();
            if (auto reverseChild = child->getState()->getReverseVertex().lock())
            {
                costToGo = reverseChild->getCost();
            }

            // The sort key is [g_F(xp) + c^(xp, xc) + h^(xc), g_F(xp) + c^(xp, xc), g_F(xp)].
            return {objective_->combineCosts(costToCome, objective_->combineCosts(edgeCost, costToGo)).value(),
                    objective_->combineCosts(costToCome, edgeCost).value(), costToCome.value()};
        }

        std::array<double, 3u> AIBITstar::computeReverseKey(const std::shared_ptr<Vertex> &parent,
                                                            const std::shared_ptr<Vertex> &child,
                                                            const ompl::base::Cost &edgeCost) const
        {
            // Assert sanity of used variables.
            assert(parent);
            assert(child);
            assert(forwardRoot_);

            // The cost to come is the cost through the tree.
            auto costToCome = parent->getCost();

            // The cost to go is the motion cost heuristic of the child to the forward root.
            auto costToGo =
                objective_->motionCostHeuristic(child->getState()->getState(), forwardRoot_->getState()->getState());

            // The sort key is [g_F(xp) + c^(xp, xc) + h^(xc), g_F(xp) + c^(xp, xc), g_F(xp)].
            return {objective_->combineCosts(costToCome, objective_->combineCosts(edgeCost, costToGo)).value(),
                    objective_->combineCosts(costToCome, edgeCost).value(), costToCome.value()};
        }

    }  // namespace geometric
}  // namespace ompl

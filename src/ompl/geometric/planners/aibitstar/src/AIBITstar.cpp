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
            if (auto forwardGoal = reverseRoot_->getTwin().lock())
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
                        edges.emplace_back(createReverseEdge(parent->getState(), vertex->getState()));
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

            // Assert some assumptions.
            assert(edge.parent->hasForwardVertex());
            assert(edge.parent->hasReverseVertex());
            assert(edge.parent->asForwardVertex()->getTwin().lock());
            assert(edge.parent->asForwardVertex()->getId() ==
                   edge.parent->asReverseVertex()->getTwin().lock()->getId());
            assert(edge.parent->asReverseVertex()->getId() ==
                   edge.parent->asForwardVertex()->getTwin().lock()->getId());
            assert(edge.child->hasReverseVertex());
            assert(edge.child->asForwardVertex()->getTwin().lock());
            assert(edge.child->asForwardVertex()->getId() == edge.child->asReverseVertex()->getTwin().lock()->getId());
            assert(edge.child->asReverseVertex()->getId() == edge.child->asForwardVertex()->getTwin().lock()->getId());

            // The parent must have a forward vertex associated with it.
            auto parentVertex = edge.parent->asForwardVertex();
            auto childVertex = edge.child->asForwardVertex();

            // The forward search is done if this edge can not possibly improve the forward path.
            if (canImproveForwardPath(edge))
            {
                // Check if the edge's parent is already the parent of the child.
                if (auto currentParent = childVertex->getParent().lock())
                {
                    if (currentParent->getId() == parentVertex->getId())
                    {
                        if (!isClosed(childVertex))
                        {
                            forwardQueue_.insert(forwardExpand(edge.child));
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
                        // Compute the true edge cost.
                        auto trueEdgeCost = objective_->motionCost(edge.parent->raw(), edge.child->raw());

                        // Check if the edge can actually improve the forward path and tree.
                        if (doesImproveForwardPath(edge, trueEdgeCost) && doesImproveForwardTree(edge, trueEdgeCost))
                        {
                            // Update the parent of the child in the forward tree.
                            childVertex->setParent(parentVertex);

                            // Update the cost.
                            childVertex->setCost(objective_->combineCosts(parentVertex->getCost(), trueEdgeCost));

                            // Add the child to the parents children.
                            parentVertex->addChild(childVertex);

                            // Expand the outgoing edges into the queue unless this state is the goal state.
                            if (edge.child->getId() != reverseRoot_->getState()->getId())
                            {
                                if (!isClosed(childVertex))
                                {
                                    forwardQueue_.insert(forwardExpand(edge.child));
                                }
                            }
                            else  // It is the goal state, update the solution.
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
                forwardQueue_.clear();
                ++searchTag_;
            }
        }

        void AIBITstar::reverseIterate()
        {
            // Ensure the reverse queue is not empty.
            assert(!reverseQueue_.empty());

            // Get the top edge from the queue.
            auto edge = reverseQueue_.pop();

            // The parent vertex must have an associated vertex in the tree.
            assert(edge.parent->hasReverseVertex());
            auto parentVertex = edge.parent->asReverseVertex();
            auto childVertex = edge.child->asReverseVertex();

            // The reverse search is done if this edge can not improve the reverse path.
            if (canImproveReversePath(edge))
            {
                if (auto currentParent = childVertex->getParent().lock())
                {
                    if (!isClosed(childVertex) && currentParent->getId() == edge.parent->asReverseVertex()->getId())
                    {
                        reverseQueue_.insert(reverseExpand(edge.child));
                        return;
                    }
                }
                if (canImproveReverseTree(edge))
                {
                    // Update the parent of the child in the reverse tree.
                    childVertex->setParent(parentVertex);

                    // Update the cost.
                    childVertex->setCost(ompl::base::Cost(edge.key[1]));

                    // Add the child to the children of the parent.
                    parentVertex->addChild(childVertex);

                    // Expand the outgoing edges into the queue unless this is the start state.
                    if (!isClosed(childVertex) && edge.child->getId() != forwardRoot_->getState()->getId())
                    {
                        reverseQueue_.insert(reverseExpand(edge.child));
                    }
                }
                if (reverseQueue_.empty())
                {
                    // Update the search tag.
                    ++searchTag_;

                    // If there could be a path, rebuild the queue or insert the outgoing edges of the start.
                    if (forwardRoot_->getTwin().lock())
                    {
                        assert(forwardRoot_->getState()->hasReverseVertex());
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
                    else  // If the start is not reached by the backward search, there is no need to continue the
                          // forward search.
                    {
                        forwardQueue_.clear();
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
                if (forwardRoot_->getTwin().lock())
                {
                    assert(forwardRoot_->getState()->hasReverseVertex());
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

        void AIBITstar::updateSolution() const
        {
            // Throw if the reverse root does not have a forward vertex.
            assert(reverseRoot_->getTwin().lock());
            assert(reverseRoot_->getState()->hasForwardVertex());
            auto current = reverseRoot_->getTwin().lock();

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
            solution.optimized_ = objective_->isSatisfied(reverseRoot_->getTwin().lock()->getCost());
            problem_->addSolutionPath(solution);
        }

        bool AIBITstar::canImproveForwardPath(const Edge &edge) const
        {
            // If the start state has not been discovered by the reverse search, the answer is always yes.
            if (auto goalVertex = reverseRoot_->getTwin().lock())
            {
                assert(reverseRoot_->getState()->hasForwardVertex());
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
            return objective_->isCostBetterThan(ompl::base::Cost(edge.key[1]),
                                                edge.child->asForwardVertex()->getCost());
        }

        bool AIBITstar::doesImproveForwardPath(const Edge &edge, const ompl::base::Cost &trueEdgeCost) const
        {
            assert(edge.child->asForwardVertex()->getTwin().lock());
            assert(edge.child->hasReverseVertex());
            assert(edge.parent->hasForwardVertex());
            if (auto reverseRootForwardVertex = reverseRoot_->getTwin().lock())
            {
                assert(reverseRoot_->getState()->hasForwardVertex());
                return objective_->isCostBetterThan(
                    objective_->combineCosts(
                        edge.parent->asForwardVertex()->getCost(),
                        objective_->combineCosts(trueEdgeCost,
                                                 edge.child->asForwardVertex()->getTwin().lock()->getCost())),
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
                objective_->combineCosts(edge.parent->asForwardVertex()->getCost(), trueEdgeCost),
                edge.child->asForwardVertex()->getCost());
        }

        bool AIBITstar::isValid(const Edge &edge) const
        {
            // Check if the edge is whitelisted.
            if (edge.parent->isWhitelisted(edge.child))
            {
                return true;
            }
            // Check if the edge is blacklisted.
            else if (edge.parent->isBlacklisted(edge.child))
            {
                return false;
            }
            // Ok, fine, we have to do work.
            else if (motionValidator_->checkMotion(edge.parent->raw(), edge.child->raw()))
            {
                // Whitelist this edge.
                edge.parent->whitelist(edge.child);
                edge.child->whitelist(edge.parent);
                return true;
            }
            else
            {
                // Blacklist this edge.
                edge.parent->blacklist(edge.child);
                edge.child->blacklist(edge.parent);
                return false;
            }
        }

        void AIBITstar::processInvalidEdge(const Edge &edge)
        {
            // Assert the edge is actually invalid.
            assert(!isValid(edge));

            // Invalidate the branch.
            auto invalidatedReverseVertex = edge.parent->asReverseVertex();
            invalidatedReverseVertex->releaseBranchFromStates();
            invalidatedReverseVertex->resetParent();
            edge.child->asReverseVertex()->removeIfChild(invalidatedReverseVertex);

            // Clear the forward queue.
            forwardQueue_.clear();

            // Register the invalid edge with the graph.
            graph_.registerInvalidEdge(edge);

            // Assert we did not invalidate the roots.
            assert(forwardRoot_->getState()->hasForwardVertex());
            assert(reverseRoot_->getState()->hasReverseVertex());
        }

        void AIBITstar::rebuildForwardQueue()
        {
            // This is going to be a bit messy. Get the edges in the queue.
            auto edges = forwardQueue_.getEdges();

            // Get the parent vertices of the edges.
            for (auto &edge : edges)
            {
                std::cout << "Rebuilding the forward queue.\n";
                edge.key = computeForwardKey(edge.parent, edge.child, edge.heuristicCost);
            }

            // All edges have an updated key, lets rebuild the queue.
            forwardQueue_.clear();
            forwardQueue_.insert(edges);
        }

        bool AIBITstar::isClosed(const std::shared_ptr<Vertex> &vertex) const
        {
            return vertex->getExpandTag() == searchTag_;
        }

        bool AIBITstar::canImproveReversePath(const Edge &edge) const
        {
            // If the start state has not been discovered by the reverse search, the answer is always yes.
            if (auto startVertex = forwardRoot_->getTwin().lock())
            {
                assert(forwardRoot_->getState()->hasReverseVertex());
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
            return objective_->isCostBetterThan(
                objective_->combineCosts(edge.parent->asReverseVertex()->getCost(), edge.heuristicCost),
                edge.child->asReverseVertex()->getCost());
        }

        std::vector<Edge> AIBITstar::forwardExpand(const std::shared_ptr<State> &state) const
        {
            // Expanding a state into the forward queue means that the state must have a parent in the forward tree.
            assert(state->hasForwardVertex());

            // Prepare the return variable.
            std::vector<Edge> outgoingEdges;

            // Get the forward vertex.
            auto forwardVertex = state->asForwardVertex();

            // Add the outgoing edges to the neighbors in the current graph.
            for (const auto &neighborState : graph_.getNeighbors(state))
            {
                outgoingEdges.emplace_back(createForwardEdge(state, neighborState));
            }

            // Add the outgoing edge to this vertex's parent in the forward tree, if it exists.
            if (forwardVertex->getId() != forwardRoot_->getId())
            {
                // If this vertex is not the forward root, it must have a parent.
                assert(forwardVertex->getParent().lock());

                // Add the edge to the reverse tree parent. This seems wrong, but if this edge isn't added and this
                // vertex becomes better connected then we could potentially miss updating the current parent.
                outgoingEdges.emplace_back(createForwardEdge(state, forwardVertex->getParent().lock()->getState()));
            }

            // Add the edges to the forward children.
            for (const auto &child : forwardVertex->getChildren())
            {
                // Add the edge to the child to the outgoing edges.
                outgoingEdges.emplace_back(createForwardEdge(state, child->getState()));
            }

            // If this state is in the reverse search tree as well, add its reverse parent and children.
            if (state->hasReverseVertex())
            {
                // Get the reverse vertex;
                auto reverseVertex = state->asReverseVertex();

                // Add the parent if it exists.
                if (auto reverseVertexParent = reverseVertex->getParent().lock())
                {
                    // Add the edge to the reverse tree parent.
                    outgoingEdges.emplace_back(createForwardEdge(state, reverseVertexParent->getState()));
                }

                for (const auto &child : reverseVertex->getChildren())
                {
                    outgoingEdges.emplace_back(createForwardEdge(state, child->getState()));
                }
            }

            // Remember that this vertex is expanded.
            forwardVertex->setExpandTag(searchTag_);

            return outgoingEdges;
        }

        std::vector<Edge> AIBITstar::reverseExpand(const std::shared_ptr<State> &state) const
        {
            // Expanding a state into the reverse queue means that the state must have a parent in the reverse tree.
            assert(state->hasReverseVertex());

            // Prepare the return variable.
            std::vector<Edge> outgoingEdges;

            // Get the reverse vertex.
            auto reverseVertex = state->asReverseVertex();

            // Get the neighbors in the current graph.
            for (const auto &neighborState : graph_.getNeighbors(state))
            {
                // Add the edge to this neighbor to the outgoing edges.
                outgoingEdges.emplace_back(createReverseEdge(state, neighborState));
            }

            // Add the outgoing edge to this vertex's parent in the reverse tree, if it exists.
            if (reverseVertex->getId() != reverseRoot_->getId())
            {
                // If this vertex is not the reverse root, it must have a parent.
                assert(reverseVertex->getParent().lock());

                // Add the edge to the reverse tree parent. This seems wrong, but if this edge isn't added and this
                // vertex becomes better connected then we could potentially miss updating the current parent.
                outgoingEdges.emplace_back(createReverseEdge(state, reverseVertex->getParent().lock()->getState()));
            }

            // Add the edge to the reverse children.
            for (const auto &child : reverseVertex->getChildren())
            {
                // Add the edge to the child to the outgoing edges.
                outgoingEdges.emplace_back(createReverseEdge(state, child->getState()));
            }

            // If this state is in the forward search tree as well, add its forward parent and children.
            if (state->hasForwardVertex())
            {
                // Get the forward vertex.
                auto forwardVertex = state->asForwardVertex();

                // Add the parent if it exists.
                if (auto forwardVertexParent = forwardVertex->getParent().lock())
                {
                    // Add the edge to the forward tree parent.
                    outgoingEdges.emplace_back(createReverseEdge(state, forwardVertexParent->getState()));
                }

                // Add the children.
                for (const auto &child : forwardVertex->getChildren())
                {
                    // Add the edge to the child to the outgoing edges.
                    outgoingEdges.emplace_back(createReverseEdge(state, child->getState()));
                }
            }

            // Remember that this vertex is expanded.
            reverseVertex->setExpandTag(searchTag_);

            return outgoingEdges;
        }

        aibitstar::Edge AIBITstar::createForwardEdge(const std::shared_ptr<aibitstar::State> &parent,
                                                     const std::shared_ptr<aibitstar::State> &child) const
        {
            // Compute the heuristic cost of the edge.
            auto edgeCost = objective_->motionCostHeuristic(parent->raw(), child->raw());

            // Return an edge.
            return aibitstar::Edge(parent, child, edgeCost, computeForwardKey(parent, child, edgeCost));
        }

        aibitstar::Edge AIBITstar::createReverseEdge(const std::shared_ptr<aibitstar::State> &parent,
                                                     const std::shared_ptr<aibitstar::State> &child) const
        {
            // Compute the heuristic cost of the edge.
            auto edgeCost = objective_->motionCostHeuristic(parent->raw(), child->raw());

            // Return an edge.
            return aibitstar::Edge(parent, child, edgeCost, computeReverseKey(parent, child, edgeCost));
        }

        std::array<double, 3u> AIBITstar::computeForwardKey(const std::shared_ptr<State> &parent,
                                                            const std::shared_ptr<State> &child,
                                                            const ompl::base::Cost &edgeCost) const
        {
            // Assert the sanity of the input.
            assert(parent);
            assert(child);
            assert(parent->hasForwardVertex());

            // The cost to come is the cost through the tree.
            auto costToCome = parent->asForwardVertex()->getCost();

            // The cost to go is infinity, if not in the reverse search tree and the corresponding cost otherwise.
            auto costToGo = child->asReverseVertex()->getCost();

            // The sort key is [g_F(xp) + c^(xp, xc) + h^(xc), g_F(xp) + c^(xp, xc), g_F(xp)].
            return {objective_->combineCosts(costToCome, objective_->combineCosts(edgeCost, costToGo)).value(),
                    objective_->combineCosts(costToCome, edgeCost).value(), costToCome.value()};
        }

        std::array<double, 3u> AIBITstar::computeReverseKey(const std::shared_ptr<State> &parent,
                                                            const std::shared_ptr<State> &child,
                                                            const ompl::base::Cost &edgeCost) const
        {
            // Assert sanity of used variables.
            assert(parent);
            assert(child);
            assert(forwardRoot_);
            assert(parent->hasReverseVertex());

            // The cost to come is the cost through the tree.
            auto costToCome = parent->asReverseVertex()->getCost();

            // The cost to go is the motion cost heuristic of the child to the forward root.
            auto costToGo = objective_->motionCostHeuristic(child->raw(), forwardRoot_->getState()->raw());

            // The sort key is [g_F(xp) + c^(xp, xc) + h^(xc), g_F(xp) + c^(xp, xc), g_F(xp)].
            return {objective_->combineCosts(costToCome, objective_->combineCosts(edgeCost, costToGo)).value(),
                    objective_->combineCosts(costToCome, edgeCost).value(), costToCome.value()};
        }

    }  // namespace geometric
}  // namespace ompl

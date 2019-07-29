/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Oxford
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

#include <algorithm>
#include <cmath>
#include <string>

#include <boost/range/adaptor/reversed.hpp>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/planners/tbdstar/TBDstar.h"
#include "ompl/util/Console.h"

using namespace std::string_literals;

namespace ompl
{
    namespace geometric
    {
        TBDstar::TBDstar(const ompl::base::SpaceInformationPtr &spaceInformation)
          : ompl::base::Planner(spaceInformation, "TBDstar")
          , forwardQueue_([](const tbdstar::Edge &lhs, const tbdstar::Edge &rhs) {
              return std::lexicographical_compare(lhs.getSortKey().begin(), lhs.getSortKey().end(),
                                                  rhs.getSortKey().begin(), rhs.getSortKey().end());
          })
          , backwardQueue_(
                [](const std::pair<double, std::shared_ptr<tbdstar::Vertex>> &lhs,
                   const std::pair<double, std::shared_ptr<tbdstar::Vertex>> &rhs) { return lhs.first < rhs.first; })
          , forwardSearchId_(std::make_shared<std::size_t>(1u))
          , backwardSearchId_(std::make_shared<std::size_t>(1u))
          , solutionCost_(std::make_shared<ompl::base::Cost>(std::numeric_limits<double>::infinity()))
        {
        }

        void TBDstar::setup()
        {
            // Check that a problem definition has been set.
            if (!static_cast<bool>(Planner::pdef_))
            {
                OMPL_ERROR("Tried to setup TBDstar, but no problem definition has been set.");
                return;
            }

            // If we were given a goal, make sure its of appropriate type.
            if (!(Planner::pdef_->getGoal()->hasType(ompl::base::GOAL_STATE) ||
                  Planner::pdef_->getGoal()->hasType(ompl::base::GOAL_STATES)))
            {
                OMPL_ERROR("%s: TBDstar is currently only implemented for goals with one or multiple distinct goal "
                           "states.",
                           Planner::getName().c_str());
                return;
            }

            // Call the base-class setup.
            Planner::setup();

            // Default to path length optimization objective if none other has been specified.
            if (!Planner::pdef_->hasOptimizationObjective())
            {
                OMPL_WARN("%s: No optimization objective has been specified. Defaulting to path length.",
                          Planner::getName().c_str());
                Planner::pdef_->setOptimizationObjective(
                    std::make_shared<ompl::base::PathLengthOptimizationObjective>(Planner::si_));
            }

            // Pull the optimization objective through the problem definition.
            optimizationObjective_ = Planner::pdef_->getOptimizationObjective();

            // Pull the motion validator through the space information.
            motionValidator_ = Planner::si_->getMotionValidator();

            // Setup a graph.
            graph_.setup(Planner::si_, Planner::pdef_, solutionCost_, forwardSearchId_, backwardSearchId_);

            // Add the start states.
            while (Planner::pis_.haveMoreStartStates())
            {
                // Get the next start state.
                const ompl::base::State *startState = Planner::pis_.nextStart();

                // Add it if it's valid.
                if (static_cast<bool>(startState))
                {
                    graph_.registerStartState(startState);
                }
            }

            // Add the goal states.
            while (Planner::pis_.haveMoreGoalStates())
            {
                const auto goalState = Planner::pis_.nextGoal();

                // Add it if it's valid.
                if (static_cast<bool>(goalState))
                {
                    graph_.registerGoalState(goalState);
                }
            }
        }

        ompl::base::PlannerStatus TBDstar::solve(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // Ensure the planner is setup.
            Planner::checkValidity();
            if (!Planner::setup_)
            {
                auto msg = Planner::name_ + " failed to setup. Has a problem definition been set?"s;
                throw ompl::Exception(msg);
            }

            // If this is the first time solve is called, insert the outgoing edges of the start into the queue.
            if (numIterations_ == 0u)
            {
                for (const auto &goal : graph_.getGoalVertices())
                {
                    goal->setCostToComeFromGoal(optimizationObjective_->identityCost());
                    auto backwardQueuePointer = backwardQueue_.insert(std::make_pair(0.0, goal));
                    goal->setBackwardQueuePointer(backwardQueuePointer);
                }
            }

            // Iterate to solve the problem.
            while (!terminationCondition)
            {
                iterate();
            }

            if (std::isfinite(solutionCost_->value()))
            {
                return {ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION};
            }
            else
            {
                return {ompl::base::PlannerStatus::StatusType::TIMEOUT};
            }
        }

        ompl::base::Cost TBDstar::bestCost() const
        {
            return *solutionCost_;
        }

        void TBDstar::getPlannerData(base::PlannerData &data) const
        {
            // Fill the planner progress properties.
            Planner::getPlannerData(data);

            // Get the vertices.
            auto vertices = graph_.getVertices();

            // Add the vertices and edges.
            for (const auto &vertex : vertices)
            {
                // Add the vertex as the right kind of vertex.
                if (graph_.isStart(vertex))
                {
                    data.addStartVertex(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()));
                }
                else if (graph_.isGoal(vertex))
                {
                    data.addGoalVertex(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()));
                }
                else
                {
                    data.addVertex(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()));
                }

                // If it has a parent, add the corresponding edge.
                if (vertex->hasForwardParent())
                {
                    data.addEdge(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()),
                                 ompl::base::PlannerDataVertex(vertex->getForwardParent()->getState(),
                                                               vertex->getForwardParent()->getId()));
                }
            }
        }

        void TBDstar::setComputeBackwardSearchHeuristic(bool computeBackwardSearchHeuristic)
        {
            computeBackwardSearchHeuristic_ = computeBackwardSearchHeuristic;
        }

        void TBDstar::rebuildForwardQueue()
        {
            std::vector<tbdstar::Edge> content;
            forwardQueue_.getContent(content);
            forwardQueue_.clear();

            for (auto &edge : content)
            {
                forwardQueue_.insert(tbdstar::Edge(edge.getParent(), edge.getChild(),
                                                   computeSortKey(edge.getParent(), edge.getChild())));
            }
        }

        std::vector<tbdstar::Edge> TBDstar::getEdgesInQueue() const
        {
            std::vector<tbdstar::Edge> edges;
            forwardQueue_.getContent(edges);
            return edges;
        }

        std::vector<std::shared_ptr<tbdstar::Vertex>> TBDstar::getVerticesInQueue() const
        {
            // Get the content from the queue.
            std::vector<std::pair<double, std::shared_ptr<tbdstar::Vertex>>> content;
            backwardQueue_.getContent(content);

            // Return the vertices.
            std::vector<std::shared_ptr<tbdstar::Vertex>> vertices;
            for (const auto &pair : content)
            {
                vertices.emplace_back(pair.second);
            }
            return vertices;
        }

        tbdstar::Edge TBDstar::getNextEdgeInQueue() const
        {
            if (!forwardQueue_.empty())
            {
                return forwardQueue_.top()->data;
            }

            return {};
        }

        std::shared_ptr<tbdstar::Vertex> TBDstar::getNextVertexInQueue() const
        {
            if (!backwardQueue_.empty())
            {
                return backwardQueue_.top()->data.second;
            }

            return {};
        }

        std::vector<std::shared_ptr<tbdstar::Vertex>> TBDstar::getVerticesInBackwardSearchTree() const
        {
            // Get all vertices from the graph.
            auto vertices = graph_.getVertices();

            // Erase the vertices that are not in the backward search tree.
            vertices.erase(std::remove_if(vertices.begin(), vertices.end(),
                                          [this](const std::shared_ptr<tbdstar::Vertex> &vertex) {
                                              return !graph_.isGoal(vertex) && !vertex->hasBackwardParent();
                                          }),
                           vertices.end());
            return vertices;
        }

        void TBDstar::iterate()
        {
            // Keep track of the number of iterations.
            ++numIterations_;

            // If there are vertices in the backward search queue, process one of them.
            if (!backwardQueue_.empty())
            {
                performBackwardSearchIteration();
            }
            else
            {
                if (!isForwardSearchStartedOnBatch_)
                {
                    // If no start vertex has finite cost to come from the goal, there is no need to start the
                    // forward search.
                    for (const auto &start : graph_.getStartVertices())
                    {
                        if (optimizationObjective_->isFinite(start->getCostToComeFromGoal()))
                        {
                            // Add the outgoing edges of the start to the queue.
                            for (const auto &start : graph_.getStartVertices())
                            {
                                start->setCostToComeFromStart(optimizationObjective_->identityCost());
                                insertOutgoingEdges(start);
                            }

                            // Remember that we've started the forward search on this batch.
                            isForwardSearchStartedOnBatch_ = true;

                            // Update the forward search id.
                            ++(*forwardSearchId_);

                            // It suffices that one start has finite cost.
                            break;
                        }
                    }
                }
                else if (forwardQueueMustBeRebuilt_)
                {
                    rebuildForwardQueue();
                    forwardQueueMustBeRebuilt_ = false;
                }
                else if (!forwardQueue_.empty())
                {
                    performForwardSearchIteration();
                }  // If both queues are empty, add new samples.
                else
                {
                    // Add new samples.
                    graph_.addSamples(batchSize_);

                    // This constitutes a new search.
                    ++(*backwardSearchId_);

                    // Add the goal vertices to the backward queue.
                    for (const auto &goal : graph_.getGoalVertices())
                    {
                        goal->setCostToComeFromGoal(optimizationObjective_->identityCost());
                        auto backwardQueuePointer = backwardQueue_.insert(std::make_pair(0.0, goal));
                        goal->setBackwardQueuePointer(backwardQueuePointer);
                    }

                    // This is a new batch, so the forward search hasn't been started.
                    isForwardSearchStartedOnBatch_ = false;
                }
            }
        }

        void TBDstar::performForwardSearchIteration()
        {
            // Get the most promising edge.
            auto &edge = forwardQueue_.top()->data;
            auto parent = edge.getParent();
            auto child = edge.getChild();
            forwardQueue_.pop();

            // If this is edge can not possibly improve our solution, the search is done.
            auto edgeCost = optimizationObjective_->motionCostHeuristic(parent->getState(), child->getState());
            auto parentCostToGoToGoal = optimizationObjective_->combineCosts(edgeCost, child->getCostToGoToGoal());
            auto pathThroughEdgeCost =
                optimizationObjective_->combineCosts(parent->getCostToComeFromStart(), parentCostToGoToGoal);

            if (!optimizationObjective_->isCostBetterThan(pathThroughEdgeCost, *solutionCost_))
            {
                forwardQueue_.clear();
                ++(*forwardSearchId_);
            }
            else if (child->hasForwardParent() && child->getForwardParent()->getId() == parent->getId())
            {
                // This is a freebie, just insert the outgoing edges of the child.
                if (!child->hasBeenExpandedDuringCurrentForwardSearch())
                {
                    insertOutgoingEdges(child);
                }
            }
            else if (optimizationObjective_->isCostBetterThan(
                         child->getCostToComeFromStart(),
                         optimizationObjective_->combineCosts(
                             parent->getCostToComeFromStart(),
                             optimizationObjective_->motionCostHeuristic(parent->getState(), child->getState()))))
            {
                // If the edge cannot improve the cost to come to the child, we're done processing it.
                return;
            }
            else if (parent->isWhitelistedAsChild(child) ||
                     motionValidator_->checkMotion(parent->getState(), child->getState()))
            {
                // Remember that this is a good edge.
                parent->whitelistAsChild(child);

                // Compute the edge cost.
                auto edgeCost = optimizationObjective_->motionCost(parent->getState(), child->getState());

                // Check if the edge can improve the cost to come to the child.
                if (optimizationObjective_->isCostBetterThan(
                        optimizationObjective_->combineCosts(parent->getCostToComeFromStart(), edgeCost),
                        child->getCostToComeFromStart()))
                {
                    // It can, so we rewire the child.
                    child->setForwardParent(parent, edgeCost);

                    // Add it to the children of the parent.
                    parent->addToForwardChildren(child);

                    // Share the good news with the whole branch.
                    child->updateCostOfForwardBranch();

                    // Check if the solution can benefit from this.
                    updateSolution();

                    // Insert the child's outgoing edges into the queue, if it hasn't been expanded yet.
                    if (!child->hasBeenExpandedDuringCurrentForwardSearch())
                    {
                        insertOutgoingEdges(child);
                    }
                }
            }
            else
            {
                parent->blacklistAsChild(child);
                if (parent->hasBackwardParent() && parent->getBackwardParent()->getId() == child->getId())
                {
                    parent->setCostToComeFromGoal(optimizationObjective_->infiniteCost());
                    parent->resetBackwardParent();
                    child->removeFromBackwardChildren(parent->getId());
                    backwardSearchUpdateVertex(parent);
                    ++(*backwardSearchId_);
                }
                forwardQueueMustBeRebuilt_ = true;
            }
        }

        void TBDstar::performBackwardSearchIteration()
        {
            // Get the most promising vertex.
            auto vertex = backwardQueue_.top()->data.second;

            // Remove it from the queue.
            backwardQueue_.pop();
            vertex->resetBackwardQueuePointer();

            // If there is currently no reason to think this vertex can be on an optimal path, clear the queue.
            if (!optimizationObjective_->isCostBetterThan(vertex->getCostToComeFromGoal(), *solutionCost_)) {
                backwardQueue_.clear();
                ++(*backwardSearchId_);
                return;
            }

            // Check if the vertex is overconsistent. g(s) < v(s).
            if (optimizationObjective_->isCostBetterThan(vertex->getCostToComeFromGoal(),
                                                         vertex->getExpandedCostToComeFromGoal()))
            {
                // Remember that this vertex is about to be expanded.
                vertex->setExpandedCostToComeFromGoal(vertex->getCostToComeFromGoal());

                // Update all successors. Start with the neighbors.
                for (const auto &neighbor : graph_.getNeighbors(vertex))
                {
                    if (neighbor->getId() != vertex->getId() && !neighbor->isBlacklistedAsChild(vertex) &&
                        !vertex->isBlacklistedAsChild(neighbor))
                    {
                        backwardSearchUpdateVertex(neighbor);
                    }
                }

                // We also need to update the forward search children.
                for (const auto &child : vertex->getForwardChildren())
                {
                    backwardSearchUpdateVertex(child);
                }

                // We also need to update the forward search parent if it exists.
                if (vertex->hasForwardParent())
                {
                    backwardSearchUpdateVertex(vertex->getForwardParent());
                }
            }
            else
            {
                vertex->setExpandedCostToComeFromGoal(optimizationObjective_->infiniteCost());
                backwardSearchUpdateVertex(vertex);
                for (const auto &neighbor : graph_.getNeighbors(vertex))
                {
                    if (neighbor->getId() != vertex->getId() && !neighbor->isBlacklistedAsChild(vertex) &&
                        !vertex->isBlacklistedAsChild(neighbor))
                    {
                        backwardSearchUpdateVertex(neighbor);
                    }
                }

                // We also need to update the forward search children.
                for (const auto &child : vertex->getForwardChildren())
                {
                    backwardSearchUpdateVertex(child);
                }

                // We also need to update the forward search parent if it exists.
                if (vertex->hasForwardParent())
                {
                    backwardSearchUpdateVertex(vertex->getForwardParent());
                }
            }
        }

        void TBDstar::backwardSearchUpdateVertex(const std::shared_ptr<tbdstar::Vertex> &vertex)
        {
            if (!graph_.isGoal(vertex))
            {
                // Get the best parent for this vertex.
                auto bestParent = vertex->getBackwardParent();
                auto bestCost = optimizationObjective_->infiniteCost();

                // Check all neighbors as defined by the graph.
                for (const auto &neighbor : graph_.getNeighbors(vertex))
                {
                    if (neighbor->getId() != vertex->getId() && !neighbor->isBlacklistedAsChild(vertex) &&
                        !vertex->isBlacklistedAsChild(neighbor))
                    {
                        auto edgeCost =
                            optimizationObjective_->motionCostHeuristic(neighbor->getState(), vertex->getState());
                        auto parentCost =
                            optimizationObjective_->combineCosts(neighbor->getCostToComeFromGoal(), edgeCost);
                        if (optimizationObjective_->isCostBetterThan(parentCost, bestCost))
                        {
                            bestParent = neighbor;
                            bestCost = parentCost;
                        }
                    }
                }

                // Check all children this vertex holds in the forward search.
                for (const auto &forwardChild : vertex->getForwardChildren())
                {
                    auto edgeCost =
                        optimizationObjective_->motionCostHeuristic(forwardChild->getState(), vertex->getState());
                    auto parentCost =
                        optimizationObjective_->combineCosts(forwardChild->getCostToComeFromGoal(), edgeCost);
                    if (optimizationObjective_->isCostBetterThan(parentCost, bestCost))
                    {
                        bestParent = forwardChild;
                        bestCost = parentCost;
                    }
                }

                // Check the parent of this vertex in the forward search.
                if (vertex->hasForwardParent())
                {
                    auto forwardParent = vertex->getForwardParent();
                    auto edgeCost =
                        optimizationObjective_->motionCostHeuristic(forwardParent->getState(), vertex->getState());
                    auto parentCost =
                        optimizationObjective_->combineCosts(forwardParent->getCostToComeFromGoal(), edgeCost);
                    if (optimizationObjective_->isCostBetterThan(parentCost, bestCost))
                    {
                        bestParent = forwardParent;
                        bestCost = parentCost;
                    }
                }

                // If this vertex is now disconnected, take special care.
                if (!optimizationObjective_->isFinite(bestCost))
                {
                    if (vertex->hasBackwardParent())
                    {
                        OMPL_WARN("Vertex %zu has backward parent %zu", vertex->getId(),
                                  vertex->getBackwardParent()->getId());
                        OMPL_WARN("Its children are:");
                        for (const auto &child : vertex->getBackwardParent()->getBackwardChildren())
                        {
                            OMPL_WARN("%zu", child->getId());
                        }
                        vertex->getBackwardParent()->removeFromBackwardChildren(vertex->getId());
                        OMPL_WARN("Removed child.");
                        vertex->resetBackwardParent();
                    }
                    return;
                }

                // Update the backward parent.
                vertex->setBackwardParent(bestParent);

                // Update the children of the parent.
                bestParent->addToBackwardChildren(vertex);

                // Set the cost to come from the goal.
                vertex->setCostToComeFromGoal(bestCost);

                // If this has made the vertex inconsistent, insert or update it in the open queue.
                if (!optimizationObjective_->isCostEquivalentTo(vertex->getCostToComeFromGoal(),
                                                                vertex->getExpandedCostToComeFromGoal()))
                {
                    insertOrUpdateInBackwardQueue(vertex);
                }
                else
                {
                    // Remove this vertex from the queue if it is in the queue.
                    auto backwardQueuePointer = vertex->getBackwardQueuePointer();
                    if (backwardQueuePointer)
                    {
                        backwardQueue_.remove(backwardQueuePointer);
                    }
                    vertex->resetBackwardQueuePointer();
                }
            }
        }

        void TBDstar::insertOrUpdateInBackwardQueue(const std::shared_ptr<tbdstar::Vertex> &vertex)
        {
            // Get the pointer to the element in the queue.
            auto element = vertex->getBackwardQueuePointer();

            // Update it if it is in the queue.
            if (element)
            {
                element->data.first = vertex->getCostToComeFromGoal().value();
                backwardQueue_.update(element);
            }
            else  // Insert it into the queue otherwise.
            {
                auto backwardQueuePointer =
                    backwardQueue_.insert(std::make_pair(vertex->getCostToComeFromGoal().value(), vertex));
                vertex->setBackwardQueuePointer(backwardQueuePointer);
            }
        }

        std::vector<std::shared_ptr<tbdstar::Vertex>>
        TBDstar::getReversePath(const std::shared_ptr<tbdstar::Vertex> &vertex) const
        {
            std::vector<std::shared_ptr<tbdstar::Vertex>> reversePath;
            auto current = vertex;
            while (!graph_.isStart(current))
            {
                reversePath.emplace_back(current);
                current = current->getForwardParent();
            }
            reversePath.emplace_back(current);
            return reversePath;
        }

        std::array<double, 3u> TBDstar::computeSortKey(const std::shared_ptr<tbdstar::Vertex> &parent,
                                                       const std::shared_ptr<tbdstar::Vertex> &child) const
        {
            // Compute the sort key [g_T(start) + c_hat(start, neighbor) + h_hat(neighbor), g_T(start) +
            // c_hat(start, neighbor), g_T(start)].
            ompl::base::Cost edgeCostHeuristic =
                optimizationObjective_->motionCostHeuristic(parent->getState(), child->getState());
            return {parent->getCostToComeFromStart().value() + edgeCostHeuristic.value() +
                        child->getCostToGoToGoal().value(),
                    edgeCostHeuristic.value() + child->getCostToGoToGoal().value(),
                    parent->getCostToComeFromStart().value()};
        }

        void TBDstar::insertOutgoingEdges(const std::shared_ptr<tbdstar::Vertex> &vertex)
        {
            // Register that this vertex is expanded on the current search.
            vertex->registerExpansionDuringForwardSearch();

            // Insert the edges to the current children.
            for (const auto &child : vertex->getForwardChildren())
            {
                forwardQueue_.insert(tbdstar::Edge(vertex, child, computeSortKey(vertex, child)));
            }

            // Insert the edges to the current neighbors.
            for (const auto &neighbor : graph_.getNeighbors(vertex))
            {
                if (neighbor->getId() != vertex->getId())
                {
                    forwardQueue_.insert(tbdstar::Edge(vertex, neighbor, computeSortKey(vertex, neighbor)));
                }
            }
        }

        void TBDstar::computeBackwardSearchHeuristic()
        {
            // Register that we started a new search.
            ++(*backwardSearchId_);

            // Create a queue for this search.
            EdgeQueue queue([](const tbdstar::Edge &lhs, const tbdstar::Edge &rhs) {
                return std::lexicographical_compare(lhs.getSortKey().begin(), lhs.getSortKey().end(),
                                                    rhs.getSortKey().begin(), rhs.getSortKey().end());
            });

            // Add the outgoing edges of the goal to the queue.
            for (const auto &goal : graph_.getGoalVertices())
            {
                // Set the backward cost to come to zero.
                goal->setCostToComeFromGoal(optimizationObjective_->identityCost());

                // Get the neighbors.
                auto neighbors = graph_.getNeighbors(goal);

                for (const auto &neighbor : neighbors)
                {
                    if (neighbor->getId() != goal->getId())
                    {
                        // Sort according to Dijkstra's.
                        queue.insert(tbdstar::Edge(
                            goal, neighbor,
                            {optimizationObjective_->motionCostHeuristic(goal->getState(), neighbor->getState())
                                 .value(),
                             0.0, 0.0}));
                    }
                }
            }

            // Process all edges. Maybe this should be a vertex queue instead?
            while (!queue.empty())
            {
                // Get the most promising edge.
                auto &edge = queue.top()->data;
                auto parent = edge.getParent();
                auto child = edge.getChild();
                queue.pop();

                // No collision checking and no computation of the actual cost here.
                ompl::base::Cost tentativeChildCost = optimizationObjective_->combineCosts(
                    parent->getCostToComeFromGoal(),
                    optimizationObjective_->motionCostHeuristic(parent->getState(), child->getState()));

                // Remember this if it improves the cost to come from the goal.
                if (optimizationObjective_->isCostBetterThan(tentativeChildCost, child->getCostToComeFromGoal()))
                {
                    // Remember the cost.
                    child->setCostToComeFromGoal(tentativeChildCost);

                    if (!child->hasBeenExpandedDuringCurrentForwardSearch())
                    {
                        child->registerExpansionDuringForwardSearch();
                        // Insert the children of the child into the queue.
                        for (const auto &grandchild : child->getForwardChildren())
                        {
                            queue.insert(
                                tbdstar::Edge(child, grandchild,
                                              {optimizationObjective_
                                                   ->combineCosts(child->getCostToComeFromGoal(),
                                                                  optimizationObjective_->motionCostHeuristic(
                                                                      child->getState(), grandchild->getState()))
                                                   .value(),
                                               0.0, 0.0}));
                        }

                        // Insert the neighbors of the child into the queue.
                        for (const auto &neighbor : graph_.getNeighbors(child))
                        {
                            if (child->getId() != neighbor->getId() && !child->isBlacklistedAsChild(neighbor))
                            {
                                queue.insert(
                                    tbdstar::Edge(child, neighbor,
                                                  {optimizationObjective_
                                                       ->combineCosts(child->getCostToComeFromGoal(),
                                                                      optimizationObjective_->motionCostHeuristic(
                                                                          child->getState(), neighbor->getState()))
                                                       .value(),
                                                   0.0, 0.0}));
                            }
                        }
                    }
                }
            }
        }

        void TBDstar::updateSolution()
        {
            // Check if any of the goals have a cost to come less than the current solution cost.
            for (const auto &goal : graph_.getGoalVertices())
            {
                if (optimizationObjective_->isCostBetterThan(goal->getCostToComeFromStart(), *solutionCost_))
                {
                    // Remember the incumbent cost.
                    *solutionCost_ = goal->getCostToComeFromStart();

                    // Create a path.
                    auto path = std::make_shared<ompl::geometric::PathGeometric>(Planner::si_);
                    auto reversePath = getReversePath(goal);
                    for (const auto &vertex : boost::adaptors::reverse(reversePath))
                    {
                        path->append(vertex->getState());
                    }

                    // Convert the path to a solution.
                    ompl::base::PlannerSolution solution(path);
                    solution.setPlannerName(Planner::name_);

                    // Set the optimized flag.
                    solution.optimized_ = optimizationObjective_->isSatisfied(*solutionCost_);

                    // Let the problem definition know that a new solution exists.
                    Planner::pdef_->addSolutionPath(solution);
                }
            }
        }

    }  // namespace geometric
}  // namespace ompl

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

#include <iostream>

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
          , backwardQueue_([](const std::pair<std::array<double, 2u>, std::shared_ptr<tbdstar::Vertex>> &lhs,
                              const std::pair<std::array<double, 2u>, std::shared_ptr<tbdstar::Vertex>> &rhs) {
              return std::lexicographical_compare(lhs.first.begin(), lhs.first.end(), rhs.first.begin(),
                                                  rhs.first.end());
          })
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
                OMPL_ERROR("Tried to setup TBD*, but no problem definition has been set.");
                return;
            }

            // If we were given a goal, make sure its of appropriate type.
            if (!(Planner::pdef_->getGoal()->hasType(ompl::base::GOAL_STATE) ||
                  Planner::pdef_->getGoal()->hasType(ompl::base::GOAL_STATES)))
            {
                OMPL_ERROR("%s: TBD* is currently only implemented for goals with one or multiple distinct goal "
                           "states.",
                           Planner::getName().c_str());
                return;
            }

            // Call the base-class setup.
            Planner::setup();

            // Default to path length optimization objective if none has been specified.
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
                auto msg = Planner::name_ + " failed to setup."s;
                throw ompl::Exception(msg);
            }

            // If this is the first time solve is called, populate the backward queue.
            if (numIterations_ == 0u)
            {
                for (const auto &goal : graph_.getGoalVertices())
                {
                    // Set the cost to come from the goal to identity cost.
                    goal->setCostToComeFromGoal(optimizationObjective_->identityCost());

                    // Create an element for the queue.
                    std::pair<std::array<double, 2u>, std::shared_ptr<tbdstar::Vertex>> element(
                        std::array<double, 2u>({computeCostToGoToStartHeuristic(goal).value(), 0.0}), goal);

                    // Insert the element into the queue and set the corresponding pointer.
                    auto backwardQueuePointer = backwardQueue_.insert(element);
                    goal->setBackwardQueuePointer(backwardQueuePointer);
                }
            }

            // Iterate to solve the problem.
            while (!terminationCondition &&
                   !(optimizationObjective_->isFinite(*solutionCost_) && stopOnFindingInitialSolution_))
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
        }  // namespace geometric

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

        void TBDstar::setBatchSize(std::size_t batchSize)
        {
            batchSize_ = batchSize;
        }

        void TBDstar::setRewireFactor(double rewireFactor)
        {
            graph_.setRewireFactor(rewireFactor);
        }

        void TBDstar::setRepairBackwardSearch(bool repairBackwardSearch)
        {
            repairBackwardSearch_ = repairBackwardSearch;
        }

        void TBDstar::setStopOnFindingInitialSolution(bool stopOnFindingInitialSolution)
        {
            stopOnFindingInitialSolution_ = stopOnFindingInitialSolution;
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

        void TBDstar::rebuildBackwardQueue()
        {
            std::vector<KeyVertexPair> content;
            backwardQueue_.getContent(content);
            backwardQueue_.clear();

            for (auto &vertex : content)
            {
                // Compute the sort key for the vertex queue.
                std::pair<std::array<double, 2u>, std::shared_ptr<tbdstar::Vertex>> element(
                    computeSortKey(vertex.second), vertex.second);
                auto backwardQueuePointer = backwardQueue_.insert(element);
                element.second->setBackwardQueuePointer(backwardQueuePointer);
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
            std::vector<std::pair<std::array<double, 2u>, std::shared_ptr<tbdstar::Vertex>>> content;
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
            if (performBackwardSearchIteration_)
            {
                if (!backwardQueue_.empty())
                {
                    performBackwardSearchIteration();
                }
                else
                {
                    performBackwardSearchIteration_ = false;
                }
            }
            else
            {
                if (!isForwardSearchStartedOnBatch_)
                {
                    // Remember that we've started the forward search on this batch.
                    isForwardSearchStartedOnBatch_ = true;

                    // If no start vertex has finite cost to come from the goal, there is no need to start the
                    // forward search.
                    for (const auto &start : graph_.getStartVertices())
                    {
                        if (optimizationObjective_->isFinite(start->getCostToComeFromGoal()))
                        {
                            // Add the outgoing edges of all start vertices to the queue.
                            for (const auto &start : graph_.getStartVertices())
                            {
                                insertOutgoingEdges(start);
                            }

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
                }
                else  // If both queues are empty, add new samples.
                {
                    // Add new samples to the graph.
                    auto newVertices = graph_.addSamples(batchSize_);

                    // This constitutes new searches.
                    backwardQueue_.clear();
                    ++(*backwardSearchId_);
                    forwardQueue_.clear();
                    ++(*forwardSearchId_);

                    // Add the goals to the backward queue.
                    for (const auto &goal : graph_.getGoalVertices())
                    {
                        auto backwardQueuePointer = backwardQueue_.insert(std::make_pair(computeSortKey(goal), goal));
                        goal->setBackwardQueuePointer(backwardQueuePointer);
                        goal->setCostToComeFromGoal(optimizationObjective_->identityCost());
                    }

                    // This is a new batch, so the searches haven't been started.
                    isForwardSearchStartedOnBatch_ = false;

                    // We will start with a backward iteration.
                    performBackwardSearchIteration_ = true;
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
                if (optimizationObjective_->isFinite(pathThroughEdgeCost) ||
                    !optimizationObjective_->isFinite(computeBestCostToComeFromGoalOfAnyStart()))
                {
                    forwardQueue_.clear();
                }
                else
                {
                    performBackwardSearchIteration_ = true;
                }
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
                // This child should be blacklisted.
                parent->blacklistAsChild(child);

                // If desired, now is the time to repair the backward search.
                if (repairBackwardSearch_)
                {
                    if (parent->hasBackwardParent() && parent->getBackwardParent()->getId() == child->getId())
                    {
                        // The parent was connected to the child through an invalid edge.
                        parent->setCostToComeFromGoal(optimizationObjective_->infiniteCost());
                        parent->resetBackwardParent();
                        child->removeFromBackwardChildren(parent->getId());

                        // This also affects all children of this vertex.
                        invalidateCostToComeFromGoalOfBackwardBranch(parent);

                        // If any of these children are in the backward queue, their sort key is outdated.
                        rebuildBackwardQueue();

                        // The parent's cost-to-come needs to be updated. This places children in open.
                        backwardSearchUpdateVertex(parent);

                        // The backward queue has to be rebuilt, because some vertices
                        performBackwardSearchIteration_ = true;

                        // This invalidates the cost-to-go estimate of the forward search.
                        forwardQueueMustBeRebuilt_ = true;
                    }
                }
            }
        }

        void TBDstar::performBackwardSearchIteration()
        {
            // Get the most promising vertex.
            auto vertex = backwardQueue_.top()->data.second;

            // Remove it from the queue.
            backwardQueue_.pop();
            vertex->resetBackwardQueuePointer();

            // The open queue should not contain consistent vertices.
            assert(!optimizationObjective_->isCostEquivalentTo(vertex->getCostToComeFromGoal(),
                                                               vertex->getExpandedCostToComeFromGoal()));

            // If any goal is underconsistent, we need to continue.
            bool underconsistentStart {false};
            for (const auto &start : graph_.getStartVertices())
            {
                if (optimizationObjective_->isCostBetterThan(start->getExpandedCostToComeFromGoal(),
                                                             start->getCostToComeFromGoal()))
                {
                    underconsistentStart = true;
                    break;
                }
            }

            // If there is currently no reason to think this vertex can be on an optimal path, clear the queue.
            if ((!underconsistentStart && !optimizationObjective_->isCostBetterThan(
                    optimizationObjective_->combineCosts(vertex->getCostToComeFromGoal(),
                                                         computeCostToGoToStartHeuristic(vertex)),
                    *solutionCost_)) ||
                optimizationObjective_->isCostBetterThan(
                    ompl::base::Cost(computeBestCostToComeFromGoalOfAnyStart().value() + 1e-6), *solutionCost_))
            {
                performBackwardSearchIteration_ = false;
                return;
            }

            // Check if the vertex is overconsistent. g(s) < v(s).
            if (optimizationObjective_->isCostBetterThan(vertex->getCostToComeFromGoal(),
                                                         vertex->getExpandedCostToComeFromGoal()))
            {
                // Register the expansion of this vertex.
                vertex->registerExpansionDuringBackwardSearch();
            }
            else
            {
                // Register the expansion of this vertex.
                vertex->registerExpansionDuringBackwardSearch();
                vertex->setExpandedCostToComeFromGoal(optimizationObjective_->infiniteCost());
                backwardSearchUpdateVertex(vertex);
            }

            // Update all successors. Start with the backward search children, because if this vertex
            // becomes the parent of a neighbor, that neighbor would be updated again as part of the
            // backward children.
            for (const auto &child : vertex->getBackwardChildren())
            {
                backwardSearchUpdateVertex(child);
            }

            // We can now process the neighbors.
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

        void TBDstar::backwardSearchUpdateVertex(const std::shared_ptr<tbdstar::Vertex> &vertex)
        {
            if (!graph_.isGoal(vertex))
            {
                // Get the best parent for this vertex.
                auto bestParent = vertex->getBackwardParent();
                auto bestCost = vertex->getCostToComeFromGoal();

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

                // Check all children this vertex hold in the backward search.
                for (const auto &backwardChild : vertex->getBackwardChildren())
                {
                    auto edgeCost =
                        optimizationObjective_->motionCostHeuristic(backwardChild->getState(), vertex->getState());
                    auto parentCost =
                        optimizationObjective_->combineCosts(backwardChild->getExpandedCostToComeFromGoal(), edgeCost);
                    if (optimizationObjective_->isCostBetterThan(parentCost, bestCost))
                    {
                        bestParent = backwardChild;
                        bestCost = parentCost;
                    }
                }

                // Check all children this vertex holds in the forward search.
                for (const auto &forwardChild : vertex->getForwardChildren())
                {
                    auto edgeCost =
                        optimizationObjective_->motionCostHeuristic(forwardChild->getState(), vertex->getState());
                    auto parentCost =
                        optimizationObjective_->combineCosts(forwardChild->getExpandedCostToComeFromGoal(), edgeCost);
                    if (optimizationObjective_->isCostBetterThan(parentCost, bestCost))
                    {
                        bestParent = forwardChild;
                        bestCost = parentCost;
                    }
                }

                // Check the parent of this vertex in the backward search.
                if (vertex->hasBackwardParent())
                {
                    auto backwardParent = vertex->getBackwardParent();
                    auto edgeCost =
                        optimizationObjective_->motionCostHeuristic(backwardParent->getState(), vertex->getState());
                    auto parentCost =
                        optimizationObjective_->combineCosts(backwardParent->getExpandedCostToComeFromGoal(), edgeCost);
                    if (optimizationObjective_->isCostBetterThan(parentCost, bestCost))
                    {
                        bestParent = backwardParent;
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
                        optimizationObjective_->combineCosts(forwardParent->getExpandedCostToComeFromGoal(), edgeCost);
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
                        vertex->getBackwardParent()->removeFromBackwardChildren(vertex->getId());
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
                        vertex->resetBackwardQueuePointer();
                    }
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
                element->data.first = computeSortKey(vertex);
                backwardQueue_.update(element);
            }
            else  // Insert it into the queue otherwise.
            {
                // Compute the sort key for the vertex queue.
                std::pair<std::array<double, 2u>, std::shared_ptr<tbdstar::Vertex>> element(computeSortKey(vertex),
                                                                                            vertex);

                // Insert the vertex into the queue, storing the corresponding pointer.
                auto backwardQueuePointer = backwardQueue_.insert(element);
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

        std::array<double, 2u> TBDstar::computeSortKey(const std::shared_ptr<tbdstar::Vertex> &vertex) const
        {
            return {optimizationObjective_
                        ->combineCosts(vertex->getCostToComeFromGoal(), computeCostToGoToStartHeuristic(vertex))
                        .value(),
                    vertex->getCostToComeFromGoal().value()};
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
                // We do not want self loops.
                if (vertex->getId() == neighbor->getId())
                {
                    continue;
                }

                // If the neighbor is the backward parent, it will explicitly be added later.
                if (vertex->hasBackwardParent() && neighbor->getId() == vertex->getBackwardParent()->getId())
                {
                    continue;
                }

                // We do not want blacklisted edges.
                if (neighbor->isBlacklistedAsChild(vertex) || vertex->isBlacklistedAsChild(neighbor))
                {
                    continue;
                }

                // If none of the above tests caught on, we can insert the edge.
                forwardQueue_.insert(tbdstar::Edge(vertex, neighbor, computeSortKey(vertex, neighbor)));
            }

            // Insert the edge to the backward search parent.
            if (vertex->hasBackwardParent())
            {
                const auto &backwardParent = vertex->getBackwardParent();
                forwardQueue_.insert(tbdstar::Edge(vertex, backwardParent, computeSortKey(vertex, backwardParent)));
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

        ompl::base::Cost TBDstar::computeCostToGoToStartHeuristic(const std::shared_ptr<tbdstar::Vertex> &vertex) const
        {
            // We need to loop over all start vertices and see which is the closest one.
            ompl::base::Cost bestCost = optimizationObjective_->infiniteCost();
            for (const auto &start : graph_.getStartVertices())
            {
                bestCost = optimizationObjective_->betterCost(
                    bestCost, optimizationObjective_->motionCostHeuristic(vertex->getState(), start->getState()));
            }
            return bestCost;
        }

        ompl::base::Cost TBDstar::computeCostToGoToGoalHeuristic(const std::shared_ptr<tbdstar::Vertex> &vertex) const
        {
            // We need to loop over all goal vertices and see which is the closest one.
            ompl::base::Cost bestCost = optimizationObjective_->infiniteCost();
            for (const auto &goal : graph_.getGoalVertices())
            {
                bestCost = optimizationObjective_->betterCost(
                    bestCost, optimizationObjective_->motionCostHeuristic(vertex->getState(), goal->getState()));
            }
            return bestCost;
        }

        ompl::base::Cost TBDstar::computeBestCostToComeFromGoalOfAnyStart() const
        {
            // We need to loop over all start vertices and see which is the closest one.
            ompl::base::Cost bestCost = optimizationObjective_->infiniteCost();
            for (const auto &start : graph_.getStartVertices())
            {
                bestCost = optimizationObjective_->betterCost(bestCost, start->getCostToComeFromGoal());
            }
            return bestCost;
        }

        void TBDstar::invalidateCostToComeFromGoalOfBackwardBranch(const std::shared_ptr<tbdstar::Vertex> &vertex)
        {
            // Update the cost of all backward children and remove from open.
            for (const auto &child : vertex->getBackwardChildren())
            {
                child->setCostToComeFromGoal(optimizationObjective_->infiniteCost());
                auto backwardQueuePointer = child->getBackwardQueuePointer();
                if (backwardQueuePointer)
                {
                    backwardQueue_.remove(backwardQueuePointer);
                    child->resetBackwardQueuePointer();
                }
                invalidateCostToComeFromGoalOfBackwardBranch(child);
            }
        }

    }  // namespace geometric
}  // namespace ompl

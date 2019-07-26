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
          , searchId_(std::make_shared<std::size_t>(1u))
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
            graph_.setup(Planner::si_, Planner::pdef_, solutionCost_, searchId_);

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
                for (const auto &start : graph_.getStartVertices())
                {
                    auto neighbors = graph_.getNeighbors(start);
                    for (const auto &neighbor : neighbors)
                    {
                        if (neighbor->getId() != start->getId())
                        {
                            forwardQueue_.insert(tbdstar::Edge(start, neighbor, computeSortKey(start, neighbor)));
                        }
                    }
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

            // OMPL_WARN("Iteration %zu | num vertices %zu", numIterations_, vertices.size());

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
                if (vertex->hasParent())
                {
                    data.addEdge(
                        ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()),
                        ompl::base::PlannerDataVertex(vertex->getParent()->getState(), vertex->getParent()->getId()));
                }
            }
        }

        void TBDstar::setComputeBackwardSearchHeuristic(bool computeBackwardSearchHeuristic)
        {
            computeBackwardSearchHeuristic_ = computeBackwardSearchHeuristic;
        }

        std::vector<tbdstar::Edge> TBDstar::getEdgesInQueue() const
        {
            std::vector<tbdstar::Edge> edges;
            forwardQueue_.getContent(edges);
            return edges;
        }

        tbdstar::Edge TBDstar::getNextEdgeInQueue() const
        {
            if (!forwardQueue_.empty())
            {
                return forwardQueue_.top()->data;
            }

            return {};
        }

        void TBDstar::iterate()
        {
            // Keep track of the number of iterations.
            ++numIterations_;

            // If there are vertices in the backward search queue, process one of them.
            if (!backwardQueue_.empty())
            {
            }  // If there are edges in the forward queue, process one of them.
            else if (!forwardQueue_.empty())
            {
                // Get the most promising edge.
                auto &edge = forwardQueue_.top()->data;
                auto parent = edge.getParent();
                auto child = edge.getChild();
                auto sortKey = edge.getSortKey();
                forwardQueue_.pop();

                // If this is edge can not possibly improve our solution, the search is done.
                if (optimizationObjective_->isCostBetterThan(*solutionCost_, ompl::base::Cost(sortKey[0])))
                {
                    forwardQueue_.clear();
                    ++(*searchId_);
                }
                else if (optimizationObjective_->isCostBetterThan(child->getCostToCome(), ompl::base::Cost(sortKey[1])))
                {
                    // If the edge cannot improve the cost to come to the child, we're done processing it.
                    return;
                }
                else if (parent->isChildWhitelisted(child) ||
                         motionValidator_->checkMotion(parent->getState(), child->getState()))
                {
                    // Remember that this is a good edge.
                    parent->whitelistAsChild(child);

                    // Compute the edge cost.
                    auto edgeCost = optimizationObjective_->motionCost(parent->getState(), child->getState());

                    // Check if the edge can improve the cost to come to the child.
                    if (optimizationObjective_->isCostBetterThan(
                            optimizationObjective_->combineCosts(parent->getCostToCome(), edgeCost),
                            child->getCostToCome()))
                    {
                        // It can, so we rewire the child.
                        child->setParent(parent, edgeCost);

                        // Share the good news with the whole branch.
                        child->updateCostOfBranch();

                        // Check if the solution can benefit from this.
                        updateSolution();
                    }

                    // Insert the child's outgoing edges into the queue, if it hasn't been expanded yet.
                    if (!child->hasBeenExpandedDuringCurrentSearch())
                    {
                        // Register that this vertex is expanded.
                        child->registerExpansionDuringCurrentSearch();

                        // Insert the vertex's current children.
                        for (const auto &grandchild : child->getChildren())
                        {
                            forwardQueue_.insert(tbdstar::Edge(child, grandchild, computeSortKey(child, grandchild)));
                        }

                        // Insert the vertex's neighbors.
                        for (const auto &neighbor : graph_.getNeighbors(child))
                        {
                            if (child->getId() != neighbor->getId() && !child->isChildBlacklisted(neighbor))
                            {
                                forwardQueue_.insert(tbdstar::Edge(child, neighbor, computeSortKey(child, neighbor)));
                            }
                        }
                    }
                }
                else
                {
                    parent->blacklistAsChild(child);
                }
            }  // If both queues are empty, add new samples.
            else
            {
                // Add new samples.
                graph_.addSamples(batchSize_);

                // Compute the backward search heuristic if desired.
                if (computeBackwardSearchHeuristic_)
                {
                    computeBackwardSearchHeuristic();
                }

                // This constitutes a new search.
                ++(*searchId_);

                // Add the outgoing edges of the start to the queue.
                for (const auto &start : graph_.getStartVertices())
                {
                    // Register the expansion on this search.
                    start->registerExpansionDuringCurrentSearch();

                    // Get the neighbors.
                    auto neighbors = graph_.getNeighbors(start);
                    for (const auto &neighbor : neighbors)
                    {
                        if (neighbor->getId() != start->getId())
                        {
                            forwardQueue_.insert(tbdstar::Edge(start, neighbor, computeSortKey(start, neighbor)));
                        }
                    }
                }
            }
        }

        std::vector<const ompl::base::State *>
        TBDstar::getReversePath(const std::shared_ptr<tbdstar::Vertex> &vertex) const
        {
            std::vector<const ompl::base::State *> reversePath;
            auto current = vertex;
            while (!graph_.isStart(current))
            {
                reversePath.emplace_back(current->getState());
                current = current->getParent();
            }
            reversePath.emplace_back(current->getState());
            return reversePath;
        }

        std::array<double, 3u> TBDstar::computeSortKey(const std::shared_ptr<tbdstar::Vertex> &parent,
                                                       const std::shared_ptr<tbdstar::Vertex> &child) const
        {
            // Compute the sort key [g_T(start) + c_hat(start, neighbor) + h_hat(neighbor), g_T(start) +
            // c_hat(start, neighbor), g_T(start)].
            ompl::base::Cost edgeCostHeuristic =
                optimizationObjective_->motionCostHeuristic(parent->getState(), child->getState());
            return {parent->getCostToCome().value() + edgeCostHeuristic.value() + child->getCostToGo().value(),
                    parent->getCostToCome().value() + edgeCostHeuristic.value(), parent->getCostToCome().value()};
        }

        void TBDstar::computeBackwardSearchHeuristic()
        {
            // Register that we started a new search.
            ++(*searchId_);

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

                    if (!child->hasBeenExpandedDuringCurrentSearch())
                    {
                        child->registerExpansionDuringCurrentSearch();
                        // Insert the children of the child into the queue.
                        for (const auto &grandchild : child->getChildren())
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
                            if (child->getId() != neighbor->getId() && !child->isChildBlacklisted(neighbor))
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
                if (optimizationObjective_->isCostBetterThan(goal->getCostToCome(), *solutionCost_))
                {
                    // Remember the incumbent cost.
                    *solutionCost_ = goal->getCostToCome();

                    // Create a path.
                    auto path = std::make_shared<ompl::geometric::PathGeometric>(Planner::si_);
                    auto reversePath = getReversePath(goal);
                    for (const auto &state : boost::adaptors::reverse(reversePath))
                    {
                        path->append(state);
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

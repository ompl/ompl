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
 *   * Neither the names of the copyright holders nor the names of its
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

#include "ompl/geometric/planners/informedtrees/AITstar.h"

#include <algorithm>
#include <cmath>
#include <string>

#include <boost/range/adaptor/reversed.hpp>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/util/Console.h"

using namespace std::string_literals;
using namespace ompl::geometric::aitstar;

namespace ompl
{
    namespace geometric
    {
        AITstar::AITstar(const ompl::base::SpaceInformationPtr &spaceInformation)
          : ompl::base::Planner(spaceInformation, "AITstar")
          , solutionCost_()
          , graph_(solutionCost_)
          , forwardQueue_([this](const auto &lhs, const auto &rhs) { return isEdgeBetter(lhs, rhs); })
          , reverseQueue_([this](const auto &lhs, const auto &rhs) { return isVertexBetter(lhs, rhs); })
        {
            // Specify AIT*'s planner specs.
            specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
            specs_.multithreaded = false;
            specs_.approximateSolutions = true;
            specs_.optimizingPaths = true;
            specs_.directed = true;
            specs_.provingSolutionNonExistence = false;
            specs_.canReportIntermediateSolutions = true;

            // Register the setting callbacks.
            declareParam<bool>("use_k_nearest", this, &AITstar::setUseKNearest, &AITstar::getUseKNearest, "0,1");
            declareParam<double>("rewire_factor", this, &AITstar::setRewireFactor, &AITstar::getRewireFactor,
                                 "1.0:0.01:3.0");
            declareParam<std::size_t>("samples_per_batch", this, &AITstar::setBatchSize, &AITstar::getBatchSize,
                                      "1:1:1000");
            declareParam<bool>("use_graph_pruning", this, &AITstar::enablePruning, &AITstar::isPruningEnabled, "0,1");
            declareParam<bool>("find_approximate_solutions", this, &AITstar::trackApproximateSolutions,
                               &AITstar::areApproximateSolutionsTracked, "0,1");
            declareParam<std::size_t>("set_max_num_goals", this, &AITstar::setMaxNumberOfGoals,
                                      &AITstar::getMaxNumberOfGoals, "1:1:1000");

            // Register the progress properties.
            addPlannerProgressProperty("iterations INTEGER", [this]() { return std::to_string(numIterations_); });
            addPlannerProgressProperty("best cost DOUBLE", [this]() { return std::to_string(solutionCost_.value()); });
            addPlannerProgressProperty("state collision checks INTEGER",
                                       [this]() { return std::to_string(graph_.getNumberOfStateCollisionChecks()); });
            addPlannerProgressProperty("edge collision checks INTEGER",
                                       [this]() { return std::to_string(numEdgeCollisionChecks_); });
            addPlannerProgressProperty("nearest neighbour calls INTEGER",
                                       [this]() { return std::to_string(graph_.getNumberOfNearestNeighborCalls()); });
        }

        void AITstar::setup()
        {
            // Call the base-class setup.
            Planner::setup();

            // Check that a problem definition has been set.
            if (static_cast<bool>(Planner::pdef_))
            {
                // Default to path length optimization objective if none has been specified.
                if (!pdef_->hasOptimizationObjective())
                {
                    OMPL_WARN("%s: No optimization objective has been specified. Defaulting to path length.",
                              Planner::getName().c_str());
                    Planner::pdef_->setOptimizationObjective(
                        std::make_shared<ompl::base::PathLengthOptimizationObjective>(Planner::si_));
                }

                if (static_cast<bool>(pdef_->getGoal()))
                {
                    // If we were given a goal, make sure its of appropriate type.
                    if (!(pdef_->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION)))
                    {
                        OMPL_ERROR("AIT* is currently only implemented for goals that can be cast to "
                                   "ompl::base::GOAL_SAMPLEABLE_GOAL_REGION.");
                        setup_ = false;
                        return;
                    }
                }

                // Pull the optimization objective through the problem definition.
                objective_ = pdef_->getOptimizationObjective();

                // Initialize the solution cost to be infinite.
                solutionCost_ = objective_->infiniteCost();
                approximateSolutionCost_ = objective_->infiniteCost();
                approximateSolutionCostToGoal_ = objective_->infiniteCost();

                // Pull the motion validator through the space information.
                motionValidator_ = si_->getMotionValidator();

                // Setup a graph.
                graph_.setup(si_, pdef_, &pis_);
            }
            else
            {
                // AIT* can't be setup without a problem definition.
                setup_ = false;
                OMPL_WARN("AIT*: Unable to setup without a problem definition.");
            }
        }

        ompl::base::PlannerStatus::StatusType AITstar::ensureSetup()
        {
            // Call the base planners validity check. This checks if the
            // planner is setup if not then it calls setup().
            checkValidity();

            // Ensure the planner is setup.
            if (!setup_)
            {
                OMPL_ERROR("%s: The planner is not setup.", name_.c_str());
                return ompl::base::PlannerStatus::StatusType::ABORT;
            }

            // Ensure the space is setup.
            if (!si_->isSetup())
            {
                OMPL_ERROR("%s: The space information is not setup.", name_.c_str());
                return ompl::base::PlannerStatus::StatusType::ABORT;
            }

            return ompl::base::PlannerStatus::StatusType::UNKNOWN;
        }

        ompl::base::PlannerStatus::StatusType
        AITstar::ensureStartAndGoalStates(const ompl::base::PlannerTerminationCondition &terminationCondition)
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

        void AITstar::clear()
        {
            graph_.clear();
            forwardQueue_.clear();
            reverseQueue_.clear();
            if (objective_)
            {
                solutionCost_ = objective_->infiniteCost();
                approximateSolutionCost_ = objective_->infiniteCost();
                approximateSolutionCostToGoal_ = objective_->infiniteCost();
            }
            numIterations_ = 0u;
            numInconsistentOrUnconnectedTargets_ = 0u;
            Planner::clear();
            setup_ = false;
        }

        ompl::base::PlannerStatus AITstar::solve(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // Ensure that the planner and state space are setup before solving.
            auto status = ensureSetup();

            // Return early if the planner or state space are not setup.
            if (status == ompl::base::PlannerStatus::StatusType::ABORT)
            {
                return status;
            }

            // Ensure that the problem has start and goal states before solving.
            status = ensureStartAndGoalStates(terminationCondition);

            // Return early if the problem cannot be solved.
            if (status == ompl::base::PlannerStatus::StatusType::INVALID_START ||
                status == ompl::base::PlannerStatus::StatusType::INVALID_GOAL)
            {
                return status;
            }

            OMPL_INFORM("%s: Solving the given planning problem. The current best solution cost is %.4f", name_.c_str(),
                        solutionCost_.value());

            // Iterate to solve the problem.
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

        ompl::base::Cost AITstar::bestCost() const
        {
            return solutionCost_;
        }

        void AITstar::getPlannerData(base::PlannerData &data) const
        {
            // base::PlannerDataVertex takes a raw pointer to a state. I want to guarantee, that the state lives as
            // long as the program lives.
            static std::set<std::shared_ptr<Vertex>,
                            std::function<bool(const std::shared_ptr<Vertex> &, const std::shared_ptr<Vertex> &)>>
                liveStates([](const auto &lhs, const auto &rhs) { return lhs->getId() < rhs->getId(); });

            // Fill the planner progress properties.
            Planner::getPlannerData(data);

            // Get the vertices.
            auto vertices = graph_.getVertices();

            // Add the vertices and edges.
            for (const auto &vertex : vertices)
            {
                // Add the vertex to the live states.
                liveStates.insert(vertex);

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

        void AITstar::setBatchSize(std::size_t batchSize)
        {
            batchSize_ = batchSize;
        }

        std::size_t AITstar::getBatchSize() const
        {
            return batchSize_;
        }

        void AITstar::setRewireFactor(double rewireFactor)
        {
            graph_.setRewireFactor(rewireFactor);
        }

        double AITstar::getRewireFactor() const
        {
            return graph_.getRewireFactor();
        }

        void AITstar::trackApproximateSolutions(bool track)
        {
            trackApproximateSolutions_ = track;
            if (!trackApproximateSolutions_)
            {
                if (static_cast<bool>(objective_))
                {
                    approximateSolutionCost_ = objective_->infiniteCost();
                    approximateSolutionCostToGoal_ = objective_->infiniteCost();
                }
            }
        }

        bool AITstar::areApproximateSolutionsTracked() const
        {
            return trackApproximateSolutions_;
        }

        void AITstar::enablePruning(bool prune)
        {
            isPruningEnabled_ = prune;
        }

        bool AITstar::isPruningEnabled() const
        {
            return isPruningEnabled_;
        }

        void AITstar::setUseKNearest(bool useKNearest)
        {
            graph_.setUseKNearest(useKNearest);
        }

        bool AITstar::getUseKNearest() const
        {
            return graph_.getUseKNearest();
        }

        void AITstar::setMaxNumberOfGoals(unsigned int numberOfGoals)
        {
            graph_.setMaxNumberOfGoals(numberOfGoals);
        }

        unsigned int AITstar::getMaxNumberOfGoals() const
        {
            return graph_.getMaxNumberOfGoals();
        }

        void AITstar::rebuildForwardQueue()
        {
            // Get all edges from the queue.
            std::vector<Edge> edges;
            forwardQueue_.getContent(edges);

            // Rebuilding the queue invalidates the incoming and outgoing lookup.
            for (const auto &edge : edges)
            {
                edge.getChild()->resetForwardQueueIncomingLookup();
                edge.getParent()->resetForwardQueueOutgoingLookup();
            }

            // Clear the queue.
            forwardQueue_.clear();
            numInconsistentOrUnconnectedTargets_ = 0u;

            // Insert all edges into the queue if they connect vertices that have been processed, otherwise store
            // them in the cache of edges that are to be inserted.
            for (auto &edge : edges)
            {
                insertOrUpdateInForwardQueue(
                    Edge(edge.getParent(), edge.getChild(), computeSortKey(edge.getParent(), edge.getChild())));
            }
        }

        void AITstar::clearForwardQueue()
        {
            std::vector<Edge> forwardQueue;
            forwardQueue_.getContent(forwardQueue);
            for (const auto &element : forwardQueue)
            {
                element.getChild()->resetForwardQueueIncomingLookup();
                element.getParent()->resetForwardQueueOutgoingLookup();
            }
            forwardQueue_.clear();
            numInconsistentOrUnconnectedTargets_ = 0u;
        }

        void AITstar::rebuildReverseQueue()
        {
            // Rebuilding the reverse queue invalidates the reverse queue pointers.
            std::vector<aitstar::KeyVertexPair> content;
            reverseQueue_.getContent(content);
            for (auto &element : content)
            {
                element.second->resetReverseQueuePointer();
            }
            reverseQueue_.clear();

            for (auto &vertex : content)
            {
                // Compute the sort key for the vertex queue.
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> element(
                    computeSortKey(vertex.second), vertex.second);
                auto reverseQueuePointer = reverseQueue_.insert(element);
                element.second->setReverseQueuePointer(reverseQueuePointer);
            }
        }

        void AITstar::clearReverseQueue()
        {
            std::vector<aitstar::KeyVertexPair> reverseQueue;
            reverseQueue_.getContent(reverseQueue);
            for (const auto &element : reverseQueue)
            {
                element.second->resetReverseQueuePointer();
            }
            reverseQueue_.clear();
        }

        void AITstar::informAboutNewSolution() const
        {
            OMPL_INFORM("%s (%u iterations): Found a new exact solution of cost %.4f. Sampled a total of %u states, %u "
                        "of which were valid samples (%.1f \%). Processed %u edges, %u of which were collision checked "
                        "(%.1f \%). The forward search tree has %u vertices, %u of which are start states. The reverse "
                        "search tree has %u vertices, %u of which are goal states.",
                        name_.c_str(), numIterations_, solutionCost_.value(), graph_.getNumberOfSampledStates(),
                        graph_.getNumberOfValidSamples(),
                        graph_.getNumberOfSampledStates() == 0u ?
                            0.0 :
                            100.0 * (static_cast<double>(graph_.getNumberOfValidSamples()) /
                                     static_cast<double>(graph_.getNumberOfSampledStates())),
                        numProcessedEdges_, numEdgeCollisionChecks_,
                        numProcessedEdges_ == 0u ? 0.0 :
                                                   100.0 * (static_cast<float>(numEdgeCollisionChecks_) /
                                                            static_cast<float>(numProcessedEdges_)),
                        countNumVerticesInForwardTree(), graph_.getStartVertices().size(),
                        countNumVerticesInReverseTree(), graph_.getGoalVertices().size());
        }

        void AITstar::informAboutPlannerStatus(ompl::base::PlannerStatus::StatusType status) const
        {
            switch (status)
            {
                case ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION:
                {
                    OMPL_INFORM("%s (%u iterations): Found an exact solution of cost %.4f.", name_.c_str(),
                                numIterations_, solutionCost_.value());
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION:
                {
                    OMPL_INFORM("%s (%u iterations): Did not find an exact solution, but found an approximate "
                                "solution "
                                "of cost %.4f which is %.4f away from a goal (in cost space).",
                                name_.c_str(), numIterations_, approximateSolutionCost_.value(),
                                approximateSolutionCostToGoal_.value());
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::TIMEOUT:
                {
                    if (trackApproximateSolutions_)
                    {
                        OMPL_INFORM("%s (%u iterations): Did not find any solution.", name_.c_str(), numIterations_);
                    }
                    else
                    {
                        OMPL_INFORM("%s (%u iterations): Did not find an exact solution, and tracking approximate "
                                    "solutions is disabled.",
                                    name_.c_str(), numIterations_);
                    }
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::INFEASIBLE:
                case ompl::base::PlannerStatus::StatusType::UNKNOWN:
                case ompl::base::PlannerStatus::StatusType::INVALID_START:
                case ompl::base::PlannerStatus::StatusType::INVALID_GOAL:
                case ompl::base::PlannerStatus::StatusType::UNRECOGNIZED_GOAL_TYPE:
                case ompl::base::PlannerStatus::StatusType::CRASH:
                case ompl::base::PlannerStatus::StatusType::ABORT:
                case ompl::base::PlannerStatus::StatusType::TYPE_COUNT:
                {
                    OMPL_INFORM("%s (%u iterations): Unable to solve the given planning problem.", name_.c_str(),
                                numIterations_);
                }
            }

            OMPL_INFORM(
                "%s (%u iterations): Sampled a total of %u states, %u of which were valid samples (%.1f \%). "
                "Processed %u edges, %u of which were collision checked (%.1f \%). The forward search tree "
                "has %u vertices. The reverse search tree has %u vertices.",
                name_.c_str(), numIterations_, graph_.getNumberOfSampledStates(), graph_.getNumberOfValidSamples(),
                graph_.getNumberOfSampledStates() == 0u ?
                    0.0 :
                    100.0 * (static_cast<double>(graph_.getNumberOfValidSamples()) /
                             static_cast<double>(graph_.getNumberOfSampledStates())),
                numProcessedEdges_, numEdgeCollisionChecks_,
                numProcessedEdges_ == 0u ?
                    0.0 :
                    100.0 * (static_cast<float>(numEdgeCollisionChecks_) / static_cast<float>(numProcessedEdges_)),
                countNumVerticesInForwardTree(), countNumVerticesInReverseTree());
        }

        void AITstar::insertGoalVerticesInReverseQueue()
        {
            for (const auto &goal : graph_.getGoalVertices())
            {
                // Set the cost to come from the goal to identity and the expanded cost to infinity.
                goal->setExpandedCostToComeFromGoal(objective_->infiniteCost());
                goal->setCostToComeFromGoal(objective_->identityCost());

                // Create an element for the queue.
                aitstar::KeyVertexPair element({computeCostToGoToStartHeuristic(goal), objective_->identityCost()},
                                               goal);

                // Insert the element into the queue and set the corresponding pointer.
                auto reverseQueuePointer = reverseQueue_.insert(element);
                goal->setReverseQueuePointer(reverseQueuePointer);
            }
        }

        void AITstar::expandStartVerticesIntoForwardQueue()
        {
            for (const auto &start : graph_.getStartVertices())
            {
                start->setCostToComeFromStart(objective_->identityCost());
                insertOrUpdateInForwardQueue(getOutgoingEdges(start));
            }
        }

        bool AITstar::continueReverseSearch() const
        {
            // Never continue the reverse search if the reverse of forward queue is empty.
            if (reverseQueue_.empty() || forwardQueue_.empty())
            {
                return false;
            }

            // Get references to the best edge and vertex in the queues.
            const auto &bestEdge = forwardQueue_.top()->data;
            const auto &bestVertex = reverseQueue_.top()->data;

            // The reverse search must be continued if the best edge has an inconsistent child state or if the best
            // vertex can potentially lead to a better solution than the best edge.
            return !((bestEdge.getChild()->isConsistent() &&
                      objective_->isCostBetterThan(bestEdge.getSortKey()[0u], bestVertex.first[0u])) ||
                     numInconsistentOrUnconnectedTargets_ == 0u);
        }

        bool AITstar::continueForwardSearch()
        {
            // Never continue the forward search if its queue is empty.
            if (forwardQueue_.empty())
            {
                return false;
            }

            // If the best edge in the forward queue has a potential total solution cost of infinity, the forward
            // search does not need to be continued. This can happen if the reverse search did not reach any target
            // state of the edges in the forward queue.
            const auto &bestEdgeCost = forwardQueue_.top()->data.getSortKey()[0u];
            if (!objective_->isFinite(bestEdgeCost))
            {
                return false;
            }

            // The forward search can be stopped once the resolution optimal solution has been found.
            return objective_->isCostBetterThan(bestEdgeCost, solutionCost_);
        }

        std::vector<Edge> AITstar::getEdgesInQueue() const
        {
            std::vector<Edge> edges;
            forwardQueue_.getContent(edges);
            return edges;
        }

        std::vector<std::shared_ptr<Vertex>> AITstar::getVerticesInQueue() const
        {
            // Get the content from the queue.
            std::vector<std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>> content;
            reverseQueue_.getContent(content);

            // Return the vertices.
            std::vector<std::shared_ptr<Vertex>> vertices;
            for (const auto &pair : content)
            {
                vertices.emplace_back(pair.second);
            }
            return vertices;
        }

        Edge AITstar::getNextEdgeInQueue() const
        {
            if (!forwardQueue_.empty())
            {
                return forwardQueue_.top()->data;
            }

            return {};
        }

        std::shared_ptr<Vertex> AITstar::getNextVertexInQueue() const
        {
            if (!reverseQueue_.empty())
            {
                return reverseQueue_.top()->data.second;
            }

            return {};
        }

        std::vector<std::shared_ptr<Vertex>> AITstar::getVerticesInReverseSearchTree() const
        {
            // Get all vertices from the graph.
            auto vertices = graph_.getVertices();

            // Erase the vertices that are not in the reverse search tree.
            vertices.erase(std::remove_if(vertices.begin(), vertices.end(),
                                          [this](const std::shared_ptr<Vertex> &vertex) {
                                              return !graph_.isGoal(vertex) && !vertex->hasReverseParent();
                                          }),
                           vertices.end());
            return vertices;
        }

        void AITstar::iterate(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // If this is the first time solve is called, populate the queues.
            if (numIterations_ == 0u)
            {
                insertGoalVerticesInReverseQueue();
                expandStartVerticesIntoForwardQueue();
            }

            // Keep track of the number of iterations.
            ++numIterations_;

            // If the reverse search needs to be continued, do that now.
            if (continueReverseSearch())
            {
                iterateReverseSearch();
            }  // If the reverse search is suspended, check whether the forward search needs to be continued.
            else if (continueForwardSearch())
            {
                iterateForwardSearch();
            }  // If neither the forward search nor the reverse search needs to be continued, add more samples.
            else
            {
                // Add new samples to the graph, respecting the termination condition.
                if (graph_.addSamples(batchSize_, terminationCondition))
                {
                    // Remove useless samples from the graph.
                    if (isPruningEnabled_)
                    {
                        graph_.prune();
                    }

                    // Clear the reverse search tree.
                    for (auto &goal : graph_.getGoalVertices())
                    {
                        invalidateCostToComeFromGoalOfReverseBranch(goal);
                    }

                    // Add new start and goal states if necessary.
                    if (pis_.haveMoreStartStates() || pis_.haveMoreGoalStates())
                    {
                        graph_.updateStartAndGoalStates(ompl::base::plannerAlwaysTerminatingCondition(), &pis_);
                    }

                    // Reinitialize the queues.
                    clearReverseQueue();
                    clearForwardQueue();
                    insertGoalVerticesInReverseQueue();
                    expandStartVerticesIntoForwardQueue();
                }
            }
        }

        void AITstar::iterateForwardSearch()
        {
            assert(!forwardQueue_.empty());

            // Get the most promising edge.
            auto parent = forwardQueue_.top()->data.getParent();
            auto child = forwardQueue_.top()->data.getChild();
            child->removeFromForwardQueueIncomingLookup(forwardQueue_.top());
            parent->removeFromForwardQueueOutgoingLookup(forwardQueue_.top());
            forwardQueue_.pop();

            // Ensure that the child is consistent and the parent isn't the goal.
            assert(child->isConsistent());
            assert(!graph_.isGoal(parent));

            // This counts as processing an edge.
            ++numProcessedEdges_;

            // If this edge is already in the forward tree, it's a freeby.
            if (child->hasForwardParent() && child->getForwardParent()->getId() == parent->getId())
            {
                insertOrUpdateInForwardQueue(getOutgoingEdges(child));
                return;
            }  // Check if this edge can possibly improve the current search tree.
            else if (objective_->isCostBetterThan(objective_->combineCosts(parent->getCostToComeFromStart(),
                                                                           objective_->motionCostHeuristic(
                                                                               parent->getState(), child->getState())),
                                                  child->getCostToComeFromStart()))
            {
                // The edge can possibly improve the solution and the path to the child. Let's check it for
                // collision.
                if (parent->isWhitelistedAsChild(child) ||
                    motionValidator_->checkMotion(parent->getState(), child->getState()))
                {
                    // Remember that this is a good edge.
                    if (!parent->isWhitelistedAsChild(child))
                    {
                        parent->whitelistAsChild(child);
                        numEdgeCollisionChecks_++;
                    }

                    // Compute the edge cost.
                    const auto edgeCost = objective_->motionCost(parent->getState(), child->getState());

                    // Check if the edge can improve the cost to come to the child.
                    if (objective_->isCostBetterThan(
                            objective_->combineCosts(parent->getCostToComeFromStart(), edgeCost),
                            child->getCostToComeFromStart()))
                    {
                        // Rewire the child.
                        child->setForwardParent(parent, edgeCost);

                        // Add it to the children of the parent.
                        parent->addToForwardChildren(child);

                        // Share the good news with the whole branch.
                        child->updateCostOfForwardBranch();

                        // Check if the solution can benefit from this.
                        updateSolution(child);

                        // Insert the child's outgoing edges into the queue.
                        insertOrUpdateInForwardQueue(getOutgoingEdges(child));
                    }
                }
                else  // This edge is in collision
                {
                    // The edge should be blacklisted in both directions.
                    parent->blacklistAsChild(child);
                    child->blacklistAsChild(parent);

                    // Repair the reverse search if this edge was in the reverse search tree.
                    if (parent->hasReverseParent() && parent->getReverseParent()->getId() == child->getId())
                    {
                        // The parent was connected to the child through an invalid edge, so we need to invalidate
                        // the branch of the reverse search tree starting from the parent.
                        invalidateCostToComeFromGoalOfReverseBranch(parent);
                    }
                }
            }
        }

        void AITstar::iterateReverseSearch()
        {
            assert(!reverseQueue_.empty());

            // Get the most promising vertex and remove it from the queue.
            auto vertex = reverseQueue_.top()->data.second;
            reverseQueue_.pop();
            vertex->resetReverseQueuePointer();

            // The open queue should not contain consistent vertices.
            assert(!vertex->isConsistent());

            // Check if the vertex is underconsistent. g[s] < v[s].
            if (objective_->isCostBetterThan(vertex->getCostToComeFromGoal(), vertex->getExpandedCostToComeFromGoal()))
            {
                // Make the vertex consistent and update the vertex.
                vertex->setExpandedCostToComeFromGoal(vertex->getCostToComeFromGoal());
                updateReverseSearchNeighbors(vertex);

                // Update the number of inconsistent targets in the forward queue.
                numInconsistentOrUnconnectedTargets_ -= vertex->getForwardQueueIncomingLookup().size();
            }
            else
            {
                // Make the vertex overconsistent.
                vertex->setExpandedCostToComeFromGoal(objective_->infiniteCost());

                // Update the vertex and its neighbors.
                updateReverseSearchVertex(vertex);
                updateReverseSearchNeighbors(vertex);
            }
        }

        bool AITstar::isEdgeBetter(const Edge &lhs, const Edge &rhs) const
        {
            return std::lexicographical_compare(
                lhs.getSortKey().cbegin(), lhs.getSortKey().cend(), rhs.getSortKey().cbegin(), rhs.getSortKey().cend(),
                [this](const auto &a, const auto &b) { return objective_->isCostBetterThan(a, b); });
        }

        bool AITstar::isVertexBetter(const aitstar::KeyVertexPair &lhs, const aitstar::KeyVertexPair &rhs) const
        {
            // If the costs of two vertices are equal then we prioritize inconsistent vertices that are targets of
            // edges in the forward queue.
            if (objective_->isCostEquivalentTo(lhs.first[0u], rhs.first[0u]) &&
                objective_->isCostEquivalentTo(lhs.first[1u], rhs.first[1u]))
            {
                return !lhs.second->getForwardQueueIncomingLookup().empty() && !lhs.second->isConsistent();
            }
            else
            {
                // Otherwise it's a regular lexicographical comparison of the keys.
                return std::lexicographical_compare(
                    lhs.first.cbegin(), lhs.first.cend(), rhs.first.cbegin(), rhs.first.cend(),
                    [this](const auto &a, const auto &b) { return objective_->isCostBetterThan(a, b); });
            }
        }

        void AITstar::updateReverseSearchVertex(const std::shared_ptr<Vertex> &vertex)
        {
            // If the vertex is a goal, there's no updating to do.
            if (graph_.isGoal(vertex))
            {
                assert(objective_->isCostEquivalentTo(vertex->getCostToComeFromGoal(), objective_->identityCost()));
                return;
            }

            // Get the best parent for this vertex.
            auto bestParent = vertex->getReverseParent();
            auto bestCost = vertex->hasReverseParent() ? vertex->getCostToComeFromGoal() : objective_->infiniteCost();

            // Check all neighbors as defined by the RGG.
            for (const auto &neighbor : graph_.getNeighbors(vertex))
            {
                if (neighbor->getId() != vertex->getId() && !neighbor->isBlacklistedAsChild(vertex) &&
                    !vertex->isBlacklistedAsChild(neighbor))
                {
                    auto edgeCost = objective_->motionCostHeuristic(neighbor->getState(), vertex->getState());
                    auto parentCost = objective_->combineCosts(neighbor->getExpandedCostToComeFromGoal(), edgeCost);
                    if (objective_->isCostBetterThan(parentCost, bestCost))
                    {
                        bestParent = neighbor;
                        bestCost = parentCost;
                    }
                }
            }

            // Check all children this vertex holds in the forward search.
            for (const auto &forwardChild : vertex->getForwardChildren())
            {
                auto edgeCost = objective_->motionCostHeuristic(forwardChild->getState(), vertex->getState());
                auto parentCost = objective_->combineCosts(forwardChild->getExpandedCostToComeFromGoal(), edgeCost);

                if (objective_->isCostBetterThan(parentCost, bestCost))
                {
                    bestParent = forwardChild;
                    bestCost = parentCost;
                }
            }

            // Check the parent of this vertex in the forward search.
            if (vertex->hasForwardParent())
            {
                auto forwardParent = vertex->getForwardParent();
                auto edgeCost = objective_->motionCostHeuristic(forwardParent->getState(), vertex->getState());
                auto parentCost = objective_->combineCosts(forwardParent->getExpandedCostToComeFromGoal(), edgeCost);

                if (objective_->isCostBetterThan(parentCost, bestCost))
                {
                    bestParent = forwardParent;
                    bestCost = parentCost;
                }
            }

            // Set the best cost as the cost-to-come from the goal.
            vertex->setCostToComeFromGoal(bestCost);

            // What happens next depends on whether the vertex is disconnected or not.
            if (objective_->isFinite(bestCost))
            {
                // The vertex is connected. Update the reverse parent.
                vertex->setReverseParent(bestParent);
                bestParent->addToReverseChildren(vertex);
            }
            else
            {
                // This vertex is now orphaned. Reset the reverse parent if the vertex had one.
                if (vertex->hasReverseParent())
                {
                    vertex->getReverseParent()->removeFromReverseChildren(vertex->getId());
                    vertex->resetReverseParent();
                }
            }

            // If this has made the vertex inconsistent, insert or update it in the open queue.
            if (!vertex->isConsistent())
            {
                insertOrUpdateInReverseQueue(vertex);
            }
            else  // Remove this vertex from the queue if it is in the queue if it is consistent.
            {
                auto reverseQueuePointer = vertex->getReverseQueuePointer();
                if (reverseQueuePointer)
                {
                    reverseQueue_.remove(reverseQueuePointer);
                    vertex->resetReverseQueuePointer();
                }
            }

            // This vertex now has a changed cost-to-come in the reverse search. All edges in the forward queue that
            // have this vertex as a target must be updated. This cannot be delayed, as whether the reverse search
            // can be suspended depends on the best edge in the forward queue.
            for (const auto &element : vertex->getForwardQueueIncomingLookup())
            {
                auto &edge = element->data;
                edge.setSortKey(computeSortKey(edge.getParent(), edge.getChild()));
                forwardQueue_.update(element);
            }
        }

        void AITstar::updateReverseSearchNeighbors(const std::shared_ptr<Vertex> &vertex)
        {
            // Start with the reverse search children, because if this vertex becomes the parent of a neighbor, that
            // neighbor would be updated again as part of the reverse children.
            for (const auto &child : vertex->getReverseChildren())
            {
                updateReverseSearchVertex(child);
            }

            // We can now process the neighbors.
            for (const auto &neighbor : graph_.getNeighbors(vertex))
            {
                if (neighbor->getId() != vertex->getId() && !neighbor->isBlacklistedAsChild(vertex) &&
                    !vertex->isBlacklistedAsChild(neighbor))
                {
                    updateReverseSearchVertex(neighbor);
                }
            }

            // We also need to update the forward search children.
            for (const auto &child : vertex->getForwardChildren())
            {
                updateReverseSearchVertex(child);
            }

            // We also need to update the forward search parent if it exists.
            if (vertex->hasForwardParent())
            {
                updateReverseSearchVertex(vertex->getForwardParent());
            }
        }

        void AITstar::insertOrUpdateInReverseQueue(const std::shared_ptr<Vertex> &vertex)
        {
            // Get the pointer to the element in the queue.
            auto element = vertex->getReverseQueuePointer();

            // Update it if it is in the queue.
            if (element)
            {
                element->data.first = computeSortKey(vertex);
                reverseQueue_.update(element);
            }
            else  // Insert it into the queue otherwise.
            {
                // Compute the sort key for the vertex queue.
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> element(computeSortKey(vertex),
                                                                                             vertex);

                // Insert the vertex into the queue, storing the corresponding pointer.
                auto reverseQueuePointer = reverseQueue_.insert(element);
                vertex->setReverseQueuePointer(reverseQueuePointer);
            }
        }

        void AITstar::insertOrUpdateInForwardQueue(const Edge &edge)
        {
            // Check if the edge is already in the queue and can be updated.
            const auto lookup = edge.getChild()->getForwardQueueIncomingLookup();
            const auto it = std::find_if(lookup.begin(), lookup.end(), [&edge](const auto element) {
                return element->data.getParent()->getId() == edge.getParent()->getId();
            });

            if (it != lookup.end())
            {
                // We used the incoming lookup of the child. Assert that it is already in the outgoing lookup of the
                // parent.
                assert(std::find_if(edge.getParent()->getForwardQueueOutgoingLookup().begin(),
                                    edge.getParent()->getForwardQueueOutgoingLookup().end(),
                                    [&edge](const auto element) {
                                        return element->data.getChild()->getId() == edge.getChild()->getId();
                                    }) != edge.getParent()->getForwardQueueOutgoingLookup().end());

                // This edge exists in the queue. If the new sort key is better than the old, we update it.
                if (isEdgeBetter(edge, (*it)->data))
                {
                    (*it)->data.setSortKey(edge.getSortKey());
                    forwardQueue_.update(*it);
                }
            }
            else
            {
                auto element = forwardQueue_.insert(edge);
                edge.getParent()->addToForwardQueueOutgoingLookup(element);
                edge.getChild()->addToForwardQueueIncomingLookup(element);

                // Incement the counter if the target is inconsistent.
                if (!edge.getChild()->isConsistent() || !objective_->isFinite(edge.getChild()->getCostToComeFromGoal()))
                {
                    ++numInconsistentOrUnconnectedTargets_;
                }
            }
        }

        void AITstar::insertOrUpdateInForwardQueue(const std::vector<Edge> &edges)
        {
            for (const auto &edge : edges)
            {
                insertOrUpdateInForwardQueue(edge);
            }
        }

        std::shared_ptr<ompl::geometric::PathGeometric>
        AITstar::getPathToVertex(const std::shared_ptr<Vertex> &vertex) const
        {
            // Create the reverse path by following the parents to the start.
            std::vector<std::shared_ptr<Vertex>> reversePath;
            auto current = vertex;
            while (!graph_.isStart(current))
            {
                reversePath.emplace_back(current);
                current = current->getForwardParent();
            }
            reversePath.emplace_back(current);

            // Reverse the reverse path to get the forward path.
            auto path = std::make_shared<ompl::geometric::PathGeometric>(Planner::si_);
            for (const auto &vertex : boost::adaptors::reverse(reversePath))
            {
                path->append(vertex->getState());
            }

            return path;
        }

        std::array<ompl::base::Cost, 3u> AITstar::computeSortKey(const std::shared_ptr<Vertex> &parent,
                                                                 const std::shared_ptr<Vertex> &child) const
        {
            // Compute the sort key [g_T(start) + c_hat(start, neighbor) + h_hat(neighbor), g_T(start) +
            // c_hat(start, neighbor), g_T(start)].
            ompl::base::Cost edgeCostHeuristic = objective_->motionCostHeuristic(parent->getState(), child->getState());
            return {
                objective_->combineCosts(objective_->combineCosts(parent->getCostToComeFromStart(), edgeCostHeuristic),
                                         child->getCostToGoToGoal()),
                objective_->combineCosts(edgeCostHeuristic, child->getCostToGoToGoal()),
                parent->getCostToComeFromStart()};
        }

        std::array<ompl::base::Cost, 2u> AITstar::computeSortKey(const std::shared_ptr<Vertex> &vertex) const
        {
            // LPA* sort key is [min(g(x), v(x)) + h(x); min(g(x), v(x))].
            return {objective_->combineCosts(objective_->betterCost(vertex->getCostToComeFromGoal(),
                                                                    vertex->getExpandedCostToComeFromGoal()),
                                             computeCostToGoToStartHeuristic(vertex)),
                    objective_->betterCost(vertex->getCostToComeFromGoal(), vertex->getExpandedCostToComeFromGoal())};
        }

        std::vector<Edge> AITstar::getOutgoingEdges(const std::shared_ptr<Vertex> &vertex) const
        {
            // Prepare the return variable.
            std::vector<Edge> outgoingEdges;

            // Insert the edges to the current children.
            for (const auto &child : vertex->getForwardChildren())
            {
                outgoingEdges.emplace_back(vertex, child, computeSortKey(vertex, child));
            }

            // Insert the edges to the current neighbors.
            for (const auto &neighbor : graph_.getNeighbors(vertex))
            {
                // We do not want self loops.
                if (vertex->getId() == neighbor->getId())
                {
                    continue;
                }

                // If the neighbor is the reverse parent, it will explicitly be added later.
                if (vertex->hasReverseParent() && neighbor->getId() == vertex->getReverseParent()->getId())
                {
                    continue;
                }

                // We do not want blacklisted edges.
                if (neighbor->isBlacklistedAsChild(vertex) || vertex->isBlacklistedAsChild(neighbor))
                {
                    continue;
                }

                // If none of the above tests caught on, we can insert the edge.
                outgoingEdges.emplace_back(vertex, neighbor, computeSortKey(vertex, neighbor));
            }

            // Insert the edge to the reverse search parent.
            if (vertex->hasReverseParent())
            {
                const auto &reverseParent = vertex->getReverseParent();
                outgoingEdges.emplace_back(vertex, reverseParent, computeSortKey(vertex, reverseParent));
            }

            return outgoingEdges;
        }

        void AITstar::updateExactSolution()
        {
            // Check if any of the goals have a cost to come less than the current solution cost.
            for (const auto &goal : graph_.getGoalVertices())
            {
                // We need to check whether the cost is better, or whether someone has removed the exact solution
                // from the problem definition.
                if (objective_->isCostBetterThan(goal->getCostToComeFromStart(), solutionCost_) ||
                    (!pdef_->hasExactSolution() && objective_->isFinite(goal->getCostToComeFromStart())))
                {
                    // Remember the incumbent cost.
                    solutionCost_ = goal->getCostToComeFromStart();

                    // Create a solution.
                    ompl::base::PlannerSolution solution(getPathToVertex(goal));
                    solution.setPlannerName(name_);

                    // Set the optimized flag.
                    solution.setOptimized(objective_, solutionCost_, objective_->isSatisfied(solutionCost_));

                    // Let the problem definition know that a new solution exists.
                    pdef_->addSolutionPath(solution);

                    // Let the user know about the new solution.
                    informAboutNewSolution();
                }
            }
        }

        void AITstar::updateApproximateSolution()
        {
            for (auto &start : graph_.getStartVertices())
            {
                start->callOnForwardBranch([this](const auto &vertex) -> void { updateApproximateSolution(vertex); });
            }
        }

        void AITstar::updateApproximateSolution(const std::shared_ptr<Vertex> &vertex)
        {
            assert(trackApproximateSolutions_);
            if (vertex->hasForwardParent() || graph_.isStart(vertex))
            {
                auto costToGoal = computeCostToGoToGoal(vertex);

                // We need to check whether this is better than the current approximate solution or whether someone
                // has removed all approximate solutions from the problem definition.
                if (objective_->isCostBetterThan(costToGoal, approximateSolutionCostToGoal_) ||
                    !pdef_->hasApproximateSolution())
                {
                    // Remember the incumbent approximate cost.
                    approximateSolutionCost_ = vertex->getCostToComeFromStart();
                    approximateSolutionCostToGoal_ = costToGoal;
                    ompl::base::PlannerSolution solution(getPathToVertex(vertex));
                    solution.setPlannerName(name_);

                    // Set the approximate flag.
                    solution.setApproximate(costToGoal.value());

                    // This solution is approximate and can not satisfy the objective.
                    solution.setOptimized(objective_, approximateSolutionCost_, false);

                    // Let the problem definition know that a new solution exists.
                    pdef_->addSolutionPath(solution);
                }
            }
        };

        ompl::base::PlannerStatus::StatusType AITstar::updateSolution()
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

        ompl::base::PlannerStatus::StatusType AITstar::updateSolution(const std::shared_ptr<Vertex> &vertex)
        {
            updateExactSolution();
            if (objective_->isFinite(solutionCost_))
            {
                return ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION;
            }
            else if (trackApproximateSolutions_)
            {
                updateApproximateSolution(vertex);
                return ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION;
            }
            else
            {
                return ompl::base::PlannerStatus::StatusType::TIMEOUT;
            }
        }

        ompl::base::Cost AITstar::computeCostToGoToStartHeuristic(const std::shared_ptr<Vertex> &vertex) const
        {
            // We need to loop over all start vertices and see which is the closest one.
            ompl::base::Cost bestCost = objective_->infiniteCost();
            for (const auto &start : graph_.getStartVertices())
            {
                bestCost = objective_->betterCost(
                    bestCost, objective_->motionCostHeuristic(vertex->getState(), start->getState()));
            }
            return bestCost;
        }

        ompl::base::Cost AITstar::computeCostToGoToGoalHeuristic(const std::shared_ptr<Vertex> &vertex) const
        {
            // We need to loop over all goal vertices and see which is the closest one.
            ompl::base::Cost bestCost = objective_->infiniteCost();
            for (const auto &goal : graph_.getGoalVertices())
            {
                bestCost = objective_->betterCost(
                    bestCost, objective_->motionCostHeuristic(vertex->getState(), goal->getState()));
            }
            return bestCost;
        }

        ompl::base::Cost AITstar::computeCostToGoToGoal(const std::shared_ptr<Vertex> &vertex) const
        {
            // We need to loop over all goal vertices and see which is the closest one.
            ompl::base::Cost bestCost = objective_->infiniteCost();
            for (const auto &goal : graph_.getGoalVertices())
            {
                bestCost =
                    objective_->betterCost(bestCost, objective_->motionCost(vertex->getState(), goal->getState()));
            }
            return bestCost;
        }

        ompl::base::Cost AITstar::computeBestCostToComeFromGoalOfAnyStart() const
        {
            // We need to loop over all start vertices and see which is the closest one.
            ompl::base::Cost bestCost = objective_->infiniteCost();
            for (const auto &start : graph_.getStartVertices())
            {
                bestCost = objective_->betterCost(bestCost, start->getCostToComeFromGoal());
            }
            return bestCost;
        }

        std::size_t AITstar::countNumVerticesInForwardTree() const
        {
            std::size_t numVerticesInForwardTree = 0u;
            auto vertices = graph_.getVertices();
            for (const auto &vertex : vertices)
            {
                if (graph_.isStart(vertex) || vertex->hasForwardParent())
                {
                    ++numVerticesInForwardTree;
                }
            }
            return numVerticesInForwardTree;
        }

        std::size_t AITstar::countNumVerticesInReverseTree() const
        {
            std::size_t numVerticesInReverseTree = 0u;
            auto vertices = graph_.getVertices();
            for (const auto &vertex : vertices)
            {
                if (graph_.isGoal(vertex) || vertex->hasReverseParent())
                {
                    ++numVerticesInReverseTree;
                }
            }
            return numVerticesInReverseTree;
        }

        void AITstar::invalidateCostToComeFromGoalOfReverseBranch(const std::shared_ptr<Vertex> &vertex)
        {
            // If this vertex is consistent before invalidation, then all incoming edges now have targets that are
            // inconsistent.
            if (vertex->isConsistent())
            {
                numInconsistentOrUnconnectedTargets_ += vertex->getForwardQueueIncomingLookup().size();
            }

            // Reset the cost to come from the goal and the reverse parent unless the vertex is itself a goal.
            if (!graph_.isGoal(vertex))
            {
                // Reset the cost to come from the goal.
                vertex->resetCostToComeFromGoal();

                // Reset the reverse parent.
                vertex->getReverseParent()->removeFromReverseChildren(vertex->getId());
                vertex->resetReverseParent();
            }

            // Reset the expanded cost to come from goal.
            vertex->resetExpandedCostToComeFromGoal();

            // Update all affected edges in the forward queue.
            for (const auto &edge : vertex->getForwardQueueIncomingLookup())
            {
                edge->data.setSortKey(computeSortKey(edge->data.getParent(), edge->data.getChild()));
                forwardQueue_.update(edge);
            }

            // Remove this vertex from the reverse search queue if it is in it.
            auto reverseQueuePointer = vertex->getReverseQueuePointer();
            if (reverseQueuePointer)
            {
                reverseQueue_.remove(reverseQueuePointer);
                vertex->resetReverseQueuePointer();
            }

            // Update the cost of all reverse children.
            for (const auto &child : vertex->getReverseChildren())
            {
                invalidateCostToComeFromGoalOfReverseBranch(child);
            }

            // Update the reverse search vertex to ensure that this vertex is placed in open if necessary.
            updateReverseSearchVertex(vertex);
        }

    }  // namespace geometric
}  // namespace ompl

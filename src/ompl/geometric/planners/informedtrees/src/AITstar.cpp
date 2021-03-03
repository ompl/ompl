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

#include <algorithm>
#include <cmath>
#include <string>

#include <boost/range/adaptor/reversed.hpp>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/planners/informedtrees/AITstar.h"
#include "ompl/util/Console.h"

using namespace std::string_literals;

namespace ompl
{
    namespace geometric
    {
        AITstar::AITstar(const ompl::base::SpaceInformationPtr &spaceInformation)
          : ompl::base::Planner(spaceInformation, "AITstar")
          , solutionCost_()
          , graph_(solutionCost_)
          , forwardQueue_([this](const aitstar::Edge &lhs, const aitstar::Edge &rhs) {
              return std::lexicographical_compare(lhs.getSortKey().cbegin(), lhs.getSortKey().cend(),
                                                  rhs.getSortKey().cbegin(), rhs.getSortKey().cend(),
                                                  [this](const ompl::base::Cost &a, const ompl::base::Cost &b) {
                                                      return objective_->isCostBetterThan(a, b);
                                                  });
          })
          , reverseQueue_(
                [this](const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<aitstar::Vertex>> &lhs,
                       const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<aitstar::Vertex>> &rhs) {
                    return std::lexicographical_compare(lhs.first.cbegin(), lhs.first.cend(), rhs.first.cbegin(),
                                                        rhs.first.cend(),
                                                        [this](const ompl::base::Cost &a, const ompl::base::Cost &b) {
                                                            return objective_->isCostBetterThan(a, b);
                                                        });
                })
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

        void AITstar::clear()
        {
            graph_.clear();
            forwardQueue_.clear();
            reverseQueue_.clear();
            solutionCost_ = objective_->infiniteCost();
            approximateSolutionCost_ = objective_->infiniteCost();
            approximateSolutionCostToGoal_ = objective_->infiniteCost();
            edgesToBeInserted_.clear();
            numIterations_ = 0u;
            performReverseSearchIteration_ = true;
            isForwardSearchStartedOnBatch_ = false;
            forwardQueueMustBeRebuilt_ = false;
            Planner::clear();
        }

        ompl::base::PlannerStatus AITstar::solve(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // The planner status to return.
            auto status = ompl::base::PlannerStatus::StatusType::UNKNOWN;

            // Ensure the planner is setup.
            Planner::checkValidity();
            if (!Planner::setup_)
            {
                OMPL_WARN("%s: Failed to setup and thus solve can not do anything meaningful.", name_.c_str());
                status = ompl::base::PlannerStatus::StatusType::ABORT;
                informAboutPlannerStatus(status);
                return status;
            }

            // If the graph currently does not have a goal state, we wait until we get one.
            if (!graph_.hasAGoalState())
            {
                graph_.updateStartAndGoalStates(terminationCondition, &pis_);
            }

            if (!graph_.hasAStartState())
            {
                OMPL_WARN("%s: No solution can be found as no start states are available", name_.c_str());
                status = ompl::base::PlannerStatus::StatusType::INVALID_START;
                informAboutPlannerStatus(status);
                return status;
            }

            // If the graph still doesn't have a goal after waiting there's nothing to solve.
            if (!graph_.hasAGoalState())
            {
                OMPL_WARN("%s: No solution can be found as no goal states are available", name_.c_str());
                status = ompl::base::PlannerStatus::StatusType::INVALID_GOAL;
                informAboutPlannerStatus(status);
                return status;
            }

            OMPL_INFORM("%s: Searching for a solution to the given planning problem. The current best solution cost is "
                        "%.4f",
                        name_.c_str(), solutionCost_.value());

            // Iterate to solve the problem.
            while (!terminationCondition && !objective_->isSatisfied(solutionCost_))
            {
                iterate(terminationCondition);
            }

            // Someone might call ProblemDefinition::clearSolutionPaths() between invokations of Planner::solve(), in
            // which case previously found solutions are not registered with the problem definition anymore.
            updateExactSolution();

            // If there are no exact solutions registered in the problem definition and we're tracking approximate
            // solutions, find the best vertex in the graph.
            if (!pdef_->hasExactSolution() && trackApproximateSolutions_)
            {
                for (const auto &vertex : graph_.getVertices())
                {
                    updateApproximateSolution(vertex);
                }
            }

            // Return the right planner status.
            if (objective_->isFinite(solutionCost_))
            {
                status = ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION;
            }
            else if (trackApproximateSolutions_ && objective_->isFinite(approximateSolutionCost_))
            {
                status = ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION;
            }
            else
            {
                status = ompl::base::PlannerStatus::StatusType::TIMEOUT;
            }

            informAboutPlannerStatus(status);
            return status;
        }

        ompl::base::Cost AITstar::bestCost() const
        {
            return solutionCost_;
        }

        void AITstar::getPlannerData(base::PlannerData &data) const
        {
            // base::PlannerDataVertex takes a raw pointer to a state. I want to guarantee, that the state lives as long
            // as the program lives.
            static std::set<
                std::shared_ptr<aitstar::Vertex>,
                std::function<bool(const std::shared_ptr<aitstar::Vertex> &, const std::shared_ptr<aitstar::Vertex> &)>>
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

        void AITstar::setRepairReverseSearch(bool repairReverseSearch)
        {
            repairReverseSearch_ = repairReverseSearch;
        }

        void AITstar::rebuildForwardQueue()
        {
            // Get all edges from the queue.
            std::vector<aitstar::Edge> edges;
            forwardQueue_.getContent(edges);

            // Rebuilding the queue invalidates the incoming and outgoing lookup.
            for (const auto &edge : edges)
            {
                edge.getChild()->resetForwardQueueIncomingLookup();
                edge.getParent()->resetForwardQueueOutgoingLookup();
            }

            // Clear the queue.
            forwardQueue_.clear();

            // Insert all edges into the queue if they connect vertices that have been processed, otherwise store them
            // in the cache of edges that are to be inserted.
            if (haveAllVerticesBeenProcessed(edges))
            {
                for (auto &edge : edges)
                {
                    insertOrUpdateInForwardQueue(aitstar::Edge(edge.getParent(), edge.getChild(),
                                                               computeSortKey(edge.getParent(), edge.getChild())));
                }
            }
            else
            {
                edgesToBeInserted_ = edges;
                performReverseSearchIteration_ = true;
            }
        }

        void AITstar::rebuildReverseQueue()
        {
            // Rebuilding the reverse queue invalidates the reverse queue pointers.
            std::vector<KeyVertexPair> content;
            reverseQueue_.getContent(content);
            for (auto &element : content)
            {
                element.second->resetReverseQueuePointer();
            }
            reverseQueue_.clear();

            for (auto &vertex : content)
            {
                // Compute the sort key for the vertex queue.
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<aitstar::Vertex>> element(
                    computeSortKey(vertex.second), vertex.second);
                auto reverseQueuePointer = reverseQueue_.insert(element);
                element.second->setReverseQueuePointer(reverseQueuePointer);
            }
        }

        void AITstar::informAboutNewSolution() const
        {
            OMPL_INFORM("%s (%u iterations): Found a new exact solution of cost %.4f. Sampled a total of %u states, %u "
                        "of which were valid samples (%.1f \%). Processed %u edges, %u of which were collision checked "
                        "(%.1f \%). The forward search tree has %u vertices. The reverse search tree has %u vertices.",
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
                        countNumVerticesInForwardTree(), countNumVerticesInReverseTree());
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
                    OMPL_INFORM("%s (%u iterations): Did not find an exact solution, but found an approximate solution "
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

        std::vector<aitstar::Edge> AITstar::getEdgesInQueue() const
        {
            std::vector<aitstar::Edge> edges;
            forwardQueue_.getContent(edges);
            return edges;
        }

        std::vector<std::shared_ptr<aitstar::Vertex>> AITstar::getVerticesInQueue() const
        {
            // Get the content from the queue.
            std::vector<std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<aitstar::Vertex>>> content;
            reverseQueue_.getContent(content);

            // Return the vertices.
            std::vector<std::shared_ptr<aitstar::Vertex>> vertices;
            for (const auto &pair : content)
            {
                vertices.emplace_back(pair.second);
            }
            return vertices;
        }

        aitstar::Edge AITstar::getNextEdgeInQueue() const
        {
            if (!forwardQueue_.empty())
            {
                return forwardQueue_.top()->data;
            }

            return {};
        }

        std::shared_ptr<aitstar::Vertex> AITstar::getNextVertexInQueue() const
        {
            if (!reverseQueue_.empty())
            {
                return reverseQueue_.top()->data.second;
            }

            return {};
        }

        std::vector<std::shared_ptr<aitstar::Vertex>> AITstar::getVerticesInReverseSearchTree() const
        {
            // Get all vertices from the graph.
            auto vertices = graph_.getVertices();

            // Erase the vertices that are not in the reverse search tree.
            vertices.erase(std::remove_if(vertices.begin(), vertices.end(),
                                          [this](const std::shared_ptr<aitstar::Vertex> &vertex) {
                                              return !graph_.isGoal(vertex) && !vertex->hasReverseParent();
                                          }),
                           vertices.end());
            return vertices;
        }

        void AITstar::iterate(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // If this is the first time solve is called, populate the reverse queue.
            if (numIterations_ == 0u)
            {
                for (const auto &goal : graph_.getGoalVertices())
                {
                    // Set the cost to come from the goal to identity cost.
                    goal->setCostToComeFromGoal(objective_->identityCost());

                    // Create an element for the queue.
                    std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<aitstar::Vertex>> element(
                        std::array<ompl::base::Cost, 2u>(
                            {computeCostToGoToStartHeuristic(goal), ompl::base::Cost(0.0)}),
                        goal);

                    // Insert the element into the queue and set the corresponding pointer.
                    auto reverseQueuePointer = reverseQueue_.insert(element);
                    goal->setReverseQueuePointer(reverseQueuePointer);
                }
            }

            // Keep track of the number of iterations.
            ++numIterations_;

            // If the algorithm is in a state that requires performing a reverse search iteration, try to perform one.
            if (performReverseSearchIteration_)
            {
                // If the reverse queue is not empty, perform a reverse search iteration.
                if (!reverseQueue_.empty())
                {
                    performReverseSearchIteration();
                }
                else
                {
                    // If the reverse queue is empty, check if there are forward edges to be inserted.
                    // Only insert forward edges that connect vertices that have been processed with the reverse search.
                    // If the reverse queue is empty and a vertex has not been processed with the reverse queue, it
                    // means that it's not in the same connected component of the RGG as the goal. We can not reach the
                    // goal from this vertex and therefore this edge can be disregarded.
                    for (const auto &edge : edgesToBeInserted_)
                    {
                        if (haveAllVerticesBeenProcessed(edge))
                        {
                            insertOrUpdateInForwardQueue(aitstar::Edge(
                                edge.getParent(), edge.getChild(), computeSortKey(edge.getParent(), edge.getChild())));
                        }
                    }
                    edgesToBeInserted_.clear();
                    performReverseSearchIteration_ = false;
                    forwardQueueMustBeRebuilt_ = true;
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
                    std::vector<aitstar::Edge> outgoingStartEdges;
                    for (const auto &start : graph_.getStartVertices())
                    {
                        if (objective_->isFinite(start->getCostToComeFromGoal()))
                        {
                            // Add the outgoing edges of all start vertices to the queue.
                            for (const auto &start : graph_.getStartVertices())
                            {
                                const auto outgoingEdges = getOutgoingEdges(start);
                                outgoingStartEdges.insert(outgoingStartEdges.end(), outgoingEdges.begin(),
                                                          outgoingEdges.end());
                            }
                        }
                    }
                    // If all vertices of the outgoing start edges have been processed, insert the edges into the
                    // forward queue. If not, remember that they are to be inserted.
                    if (haveAllVerticesBeenProcessed(outgoingStartEdges))
                    {
                        for (const auto &edge : outgoingStartEdges)
                        {
                            insertOrUpdateInForwardQueue(edge);
                        }
                    }
                    else
                    {
                        assert(edgesToBeInserted_.empty());
                        edgesToBeInserted_ = outgoingStartEdges;
                        performReverseSearchIteration_ = true;
                    }
                }
                else if (forwardQueueMustBeRebuilt_)
                {
                    // Rebuild the forwared queue if necessary.
                    rebuildForwardQueue();
                    forwardQueueMustBeRebuilt_ = false;
                }
                else if (!forwardQueue_.empty())
                {
                    // If the forward queue is not empty, perform a forward search iteration.
                    performForwardSearchIteration();
                }
                else  // We should not perform a reverse search iteration and the forward queue is empty. Add more
                      // samples.
                {
                    // Add new samples to the graph.
                    if (graph_.addSamples(batchSize_, terminationCondition))
                    {
                        // Clear the reverse queue.
                        std::vector<std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<aitstar::Vertex>>>
                            reverseQueue;
                        reverseQueue_.getContent(reverseQueue);
                        for (const auto &element : reverseQueue)
                        {
                            element.second->resetReverseQueuePointer();
                        }
                        reverseQueue_.clear();

                        // Clear the forward queue.
                        std::vector<aitstar::Edge> forwardQueue;
                        forwardQueue_.getContent(forwardQueue);
                        for (const auto &element : forwardQueue)
                        {
                            element.getChild()->resetForwardQueueIncomingLookup();
                            element.getParent()->resetForwardQueueOutgoingLookup();
                        }
                        forwardQueue_.clear();

                        // Clear the cache of edges to be inserted.
                        edgesToBeInserted_.clear();

                        // Remove useless samples from the graph.
                        if (isPruningEnabled_)
                        {
                            graph_.prune();
                        }

                        // Add new start and goal states if necessary.
                        if (pis_.haveMoreStartStates() || pis_.haveMoreGoalStates())
                        {
                            graph_.updateStartAndGoalStates(ompl::base::plannerAlwaysTerminatingCondition(), &pis_);
                        }

                        // Add the goals to the reverse queue.
                        for (const auto &goal : graph_.getGoalVertices())
                        {
                            goal->setCostToComeFromGoal(objective_->identityCost());
                            auto reverseQueuePointer = reverseQueue_.insert(std::make_pair(computeSortKey(goal), goal));
                            goal->setReverseQueuePointer(reverseQueuePointer);
                        }

                        // This is a new batch, so the search hasn't been started.
                        isForwardSearchStartedOnBatch_ = false;

                        // We have to update the heuristic. Start with a reverse iteration.
                        performReverseSearchIteration_ = true;
                    }
                }
            }
        }

        void AITstar::performForwardSearchIteration()
        {
            // We should never perform a forward search iteration while there are still edges to be inserted.
            assert(edgesToBeInserted_.empty());

            // Get the most promising edge.
            auto &edge = forwardQueue_.top()->data;
            auto parent = edge.getParent();
            auto child = edge.getChild();

            // Make sure the edge is sane
            assert(child->hasReverseParent() || graph_.isGoal(child));
            assert(parent->hasReverseParent() || graph_.isGoal(parent));

            // Remove the edge from the incoming and outgoing lookups.
            child->removeFromForwardQueueIncomingLookup(forwardQueue_.top());
            parent->removeFromForwardQueueOutgoingLookup(forwardQueue_.top());

            // Remove the edge from the queue.
            forwardQueue_.pop();

            // This counts as processing an edge.
            ++numProcessedEdges_;

            // Register that an outgoing edge of the parent has been popped from the queue. This means that the parent
            // has optimal cost-to-come for the current approximation.
            parent->registerPoppedOutgoingEdgeDuringForwardSearch();

            // If this is edge can not possibly improve our solution, the search is done.
            auto edgeCost = objective_->motionCostHeuristic(parent->getState(), child->getState());
            auto parentCostToGoToGoal = objective_->combineCosts(edgeCost, child->getCostToGoToGoal());
            auto pathThroughEdgeCost = objective_->combineCosts(parent->getCostToComeFromStart(), parentCostToGoToGoal);
            if (!objective_->isCostBetterThan(pathThroughEdgeCost, solutionCost_))
            {
                if (objective_->isFinite(pathThroughEdgeCost) ||
                    !objective_->isFinite(computeBestCostToComeFromGoalOfAnyStart()))
                {
                    std::vector<aitstar::Edge> edges;
                    forwardQueue_.getContent(edges);
                    for (const auto &edge : edges)
                    {
                        edge.getChild()->resetForwardQueueIncomingLookup();
                        edge.getParent()->resetForwardQueueOutgoingLookup();
                    }
                    forwardQueue_.clear();
                }
                else
                {
                    performReverseSearchIteration_ = true;
                }
            }  // This edge can improve the solution. Check if it's already in the reverse search tree.
            else if (child->hasForwardParent() && child->getForwardParent()->getId() == parent->getId())
            {
                // This is a freebie, just insert the outgoing edges of the child.
                auto edges = getOutgoingEdges(child);
                if (haveAllVerticesBeenProcessed(edges))
                {
                    for (const auto &edge : edges)
                    {
                        insertOrUpdateInForwardQueue(edge);
                    }
                }
                else
                {
                    edgesToBeInserted_ = edges;
                    performReverseSearchIteration_ = true;
                    return;
                }
            }  // This edge can improve the solution and is not already in the reverse search tree.
            else if (objective_->isCostBetterThan(child->getCostToComeFromStart(),
                                                  objective_->combineCosts(parent->getCostToComeFromStart(),
                                                                           objective_->motionCostHeuristic(
                                                                               parent->getState(), child->getState()))))
            {
                // If the edge cannot improve the cost to come to the child, we're done processing it.
                return;
            }  // The edge can possibly improve the solution and the path to the child. Let's check it for collision.
            else if (parent->isWhitelistedAsChild(child) ||
                     motionValidator_->checkMotion(parent->getState(), child->getState()))
            {
                // Remember that this is a good edge.
                if (!parent->isWhitelistedAsChild(child))
                {
                    parent->whitelistAsChild(child);
                    numEdgeCollisionChecks_++;
                }

                // Compute the edge cost.
                auto edgeCost = objective_->motionCost(parent->getState(), child->getState());

                // Check if the edge can improve the cost to come to the child.
                if (objective_->isCostBetterThan(objective_->combineCosts(parent->getCostToComeFromStart(), edgeCost),
                                                 child->getCostToComeFromStart()))
                {
                    // If the child has already been expanded during the current forward search, something's fishy.
                    assert(!child->hasHadOutgoingEdgePoppedDuringCurrentForwardSearch());

                    // Rewire the child.
                    child->setForwardParent(parent, edgeCost);

                    // Add it to the children of the parent.
                    parent->addToForwardChildren(child);

                    // Share the good news with the whole branch.
                    child->updateCostOfForwardBranch();

                    // Check if the solution can benefit from this.
                    updateExactSolution();

                    // If we don't have an exact solution but are tracking approximate solutions, see if the child is
                    // the best approximate solution so far.
                    if (!pdef_->hasExactSolution() && trackApproximateSolutions_)
                    {
                        updateApproximateSolution(child);
                    }

                    // Insert the child's outgoing edges into the queue.
                    auto edges = getOutgoingEdges(child);
                    if (haveAllVerticesBeenProcessed(edges))
                    {
                        for (const auto &edge : edges)
                        {
                            insertOrUpdateInForwardQueue(edge);
                        }
                    }
                    else
                    {
                        edgesToBeInserted_ = edges;
                        performReverseSearchIteration_ = true;
                        return;
                    }
                }
            }
            else
            {
                // This child should be blacklisted.
                parent->blacklistAsChild(child);

                // If desired, now is the time to repair the reverse search.
                if (repairReverseSearch_)
                {
                    if (parent->hasReverseParent() && parent->getReverseParent()->getId() == child->getId())
                    {
                        // The parent was connected to the child through an invalid edge.
                        parent->setCostToComeFromGoal(objective_->infiniteCost());
                        parent->setExpandedCostToComeFromGoal(objective_->infiniteCost());
                        parent->resetReverseParent();
                        child->removeFromReverseChildren(parent->getId());

                        // This also affects all children of this vertex.
                        invalidateCostToComeFromGoalOfReverseBranch(parent);

                        // If any of these children are in the reverse queue, their sort key is outdated.
                        rebuildReverseQueue();

                        // The parent's cost-to-come needs to be updated. This places children in open.
                        reverseSearchUpdateVertex(parent);

                        // If the reverse queue is empty, this means we have to add new samples.
                        if (reverseQueue_.empty())
                        {
                            std::vector<aitstar::Edge> edges;
                            forwardQueue_.getContent(edges);
                            for (const auto &edge : edges)
                            {
                                edge.getChild()->resetForwardQueueIncomingLookup();
                                edge.getParent()->resetForwardQueueOutgoingLookup();
                            }
                            forwardQueue_.clear();
                        }
                        else
                        {
                            performReverseSearchIteration_ = true;
                        }
                    }
                }
            }
        }

        void AITstar::performReverseSearchIteration()
        {
            assert(!reverseQueue_.empty());

            // Get the most promising vertex.
            auto vertex = reverseQueue_.top()->data.second;

            // Remove it from the queue.
            reverseQueue_.pop();
            vertex->resetReverseQueuePointer();

            // The open queue should not contain consistent vertices.
            assert((!objective_->isFinite(vertex->getCostToComeFromGoal()) &&
                    !objective_->isFinite(vertex->getExpandedCostToComeFromGoal())) ||
                   (!objective_->isCostEquivalentTo(vertex->getCostToComeFromGoal(),
                                                    vertex->getExpandedCostToComeFromGoal())));

            // If any goal is underconsistent, we need to continue.
            bool underconsistentStart{false};
            for (const auto &start : graph_.getStartVertices())
            {
                if (objective_->isCostBetterThan(start->getExpandedCostToComeFromGoal(),
                                                 start->getCostToComeFromGoal()))
                {
                    underconsistentStart = true;
                    break;
                }
            }

            // If there is currently no reason to think this vertex can be on an optimal path, clear the queue.
            if (edgesToBeInserted_.empty() &&
                ((!underconsistentStart &&
                  !objective_->isCostBetterThan(objective_->combineCosts(vertex->getCostToComeFromGoal(),
                                                                         computeCostToGoToStartHeuristic(vertex)),
                                                solutionCost_)) ||
                 objective_->isCostBetterThan(
                     ompl::base::Cost(computeBestCostToComeFromGoalOfAnyStart().value() + 1e-6), solutionCost_)))
            {
                // This invalidates the cost-to-go estimate of the forward search.
                performReverseSearchIteration_ = false;
                forwardQueueMustBeRebuilt_ = true;
                vertex->registerExpansionDuringReverseSearch();
                return;
            }

            // Check if the vertex is overconsistent. g(s) < v(s).
            if (objective_->isCostBetterThan(vertex->getCostToComeFromGoal(), vertex->getExpandedCostToComeFromGoal()))
            {
                // Register the expansion of this vertex.
                vertex->registerExpansionDuringReverseSearch();
            }
            else
            {
                // Register the expansion of this vertex.
                vertex->registerExpansionDuringReverseSearch();
                vertex->setExpandedCostToComeFromGoal(objective_->infiniteCost());
                reverseSearchUpdateVertex(vertex);
            }

            // Update all successors. Start with the reverse search children, because if this vertex
            // becomes the parent of a neighbor, that neighbor would be updated again as part of the
            // reverse children.
            for (const auto &child : vertex->getReverseChildren())
            {
                reverseSearchUpdateVertex(child);
            }

            // We can now process the neighbors.
            for (const auto &neighbor : graph_.getNeighbors(vertex))
            {
                if (neighbor->getId() != vertex->getId() && !neighbor->isBlacklistedAsChild(vertex) &&
                    !vertex->isBlacklistedAsChild(neighbor))
                {
                    reverseSearchUpdateVertex(neighbor);
                }
            }

            // We also need to update the forward search children.
            for (const auto &child : vertex->getForwardChildren())
            {
                reverseSearchUpdateVertex(child);
            }

            // We also need to update the forward search parent if it exists.
            if (vertex->hasForwardParent())
            {
                reverseSearchUpdateVertex(vertex->getForwardParent());
            }

            if (!edgesToBeInserted_.empty())
            {
                if (haveAllVerticesBeenProcessed(edgesToBeInserted_))
                {
                    for (std::size_t i = 0u; i < edgesToBeInserted_.size(); ++i)
                    {
                        auto &edge = edgesToBeInserted_.at(i);
                        edge.setSortKey(computeSortKey(edge.getParent(), edge.getChild()));
                        insertOrUpdateInForwardQueue(edge);
                    }
                    edgesToBeInserted_.clear();
                }
            }
        }

        void AITstar::reverseSearchUpdateVertex(const std::shared_ptr<aitstar::Vertex> &vertex)
        {
            if (!graph_.isGoal(vertex))
            {
                // Get the best parent for this vertex.
                auto bestParent = vertex->getReverseParent();
                auto bestCost =
                    vertex->hasReverseParent() ? vertex->getCostToComeFromGoal() : objective_->infiniteCost();

                // Check all neighbors as defined by the graph.
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
                    auto parentCost =
                        objective_->combineCosts(forwardParent->getExpandedCostToComeFromGoal(), edgeCost);

                    if (objective_->isCostBetterThan(parentCost, bestCost))
                    {
                        bestParent = forwardParent;
                        bestCost = parentCost;
                    }
                }

                // Check the parent of this vertex in the reverse search.
                if (vertex->hasReverseParent())
                {
                    auto reverseParent = vertex->getReverseParent();
                    auto edgeCost = objective_->motionCostHeuristic(reverseParent->getState(), vertex->getState());
                    auto parentCost =
                        objective_->combineCosts(reverseParent->getExpandedCostToComeFromGoal(), edgeCost);

                    if (objective_->isCostBetterThan(parentCost, bestCost))
                    {
                        bestParent = reverseParent;
                        bestCost = parentCost;
                    }
                }

                // If this vertex is now disconnected, take special care.
                if (!objective_->isFinite(bestCost))
                {
                    // Reset the reverse parent if the vertex has one.
                    if (vertex->hasReverseParent())
                    {
                        vertex->getReverseParent()->removeFromReverseChildren(vertex->getId());
                        vertex->resetReverseParent();
                    }

                    // Invalidate the branch in the reverse search tree that is rooted at this vertex.
                    vertex->setCostToComeFromGoal(objective_->infiniteCost());
                    vertex->setExpandedCostToComeFromGoal(objective_->infiniteCost());
                    auto affectedVertices = vertex->invalidateReverseBranch();

                    // Remove the affected edges from the forward queue, placing them in the edge cache.
                    for (const auto &affectedVertex : affectedVertices)
                    {
                        auto forwardQueueIncomingLookup = affectedVertex.lock()->getForwardQueueIncomingLookup();
                        for (const auto &element : forwardQueueIncomingLookup)
                        {
                            edgesToBeInserted_.emplace_back(element->data);
                            element->data.getParent()->removeFromForwardQueueOutgoingLookup(element);
                            forwardQueue_.remove(element);
                        }
                        affectedVertex.lock()->resetForwardQueueIncomingLookup();

                        auto forwardQueueOutgoingLookup = affectedVertex.lock()->getForwardQueueOutgoingLookup();
                        for (const auto &element : forwardQueueOutgoingLookup)
                        {
                            edgesToBeInserted_.emplace_back(element->data);
                            element->data.getChild()->removeFromForwardQueueIncomingLookup(element);
                            forwardQueue_.remove(element);
                        }
                        affectedVertex.lock()->resetForwardQueueOutgoingLookup();
                    }

                    // Remove appropriate edges from the forward queue that target the root of the branch.
                    auto vertexForwardQueueIncomingLookup = vertex->getForwardQueueIncomingLookup();
                    for (const auto &element : vertexForwardQueueIncomingLookup)
                    {
                        auto &edge = element->data;
                        auto it = std::find_if(affectedVertices.begin(), affectedVertices.end(),
                                               [edge](const auto &affectedVertex) {
                                                   return affectedVertex.lock()->getId() == edge.getParent()->getId();
                                               });
                        if (it != affectedVertices.end())
                        {
                            edgesToBeInserted_.emplace_back(element->data);
                            vertex->removeFromForwardQueueIncomingLookup(element);
                            element->data.getParent()->removeFromForwardQueueOutgoingLookup(element);
                            forwardQueue_.remove(element);
                        }
                    }

                    // Remove appropriate edges from the forward queue that target the root of the branch.
                    auto vertexForwardQueueOutgoingLookup = vertex->getForwardQueueOutgoingLookup();
                    for (const auto &element : vertexForwardQueueOutgoingLookup)
                    {
                        edgesToBeInserted_.emplace_back(element->data);
                        vertex->removeFromForwardQueueOutgoingLookup(element);
                        element->data.getChild()->removeFromForwardQueueIncomingLookup(element);
                        forwardQueue_.remove(element);
                    }

                    // Check update the invalidated vertices and insert them in open if they become connected to the
                    // tree.
                    for (const auto &affectedVertex : affectedVertices)
                    {
                        auto affectedVertexPtr = affectedVertex.lock();

                        reverseSearchUpdateVertex(affectedVertexPtr);
                        if (affectedVertex.lock()->hasReverseParent())
                        {
                            insertOrUpdateInReverseQueue(affectedVertexPtr);
                            affectedVertexPtr->setExpandedCostToComeFromGoal(objective_->infiniteCost());
                        }
                    }

                    return;
                }

                // Update the reverse parent.
                vertex->setReverseParent(bestParent);

                // Update the children of the parent.
                bestParent->addToReverseChildren(vertex);

                // Set the cost to come from the goal.
                vertex->setCostToComeFromGoal(bestCost);

                // If this has made the vertex inconsistent, insert or update it in the open queue.
                if (!objective_->isCostEquivalentTo(vertex->getCostToComeFromGoal(),
                                                    vertex->getExpandedCostToComeFromGoal()))
                {
                    insertOrUpdateInReverseQueue(vertex);
                }
                else
                {
                    // Remove this vertex from the queue if it is in the queue.
                    auto reverseQueuePointer = vertex->getReverseQueuePointer();
                    if (reverseQueuePointer)
                    {
                        reverseQueue_.remove(reverseQueuePointer);
                        vertex->resetReverseQueuePointer();
                    }
                }
            }
        }

        void AITstar::insertOrUpdateInReverseQueue(const std::shared_ptr<aitstar::Vertex> &vertex)
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
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<aitstar::Vertex>> element(
                    computeSortKey(vertex), vertex);

                // Insert the vertex into the queue, storing the corresponding pointer.
                auto reverseQueuePointer = reverseQueue_.insert(element);
                vertex->setReverseQueuePointer(reverseQueuePointer);
            }
        }

        void AITstar::insertOrUpdateInForwardQueue(const aitstar::Edge &edge)
        {
            // Check if the edge is already in the queue and can be updated.
            auto lookup = edge.getChild()->getForwardQueueIncomingLookup();
            auto it = std::find_if(lookup.begin(), lookup.end(), [&edge](const auto element) {
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
                (*it)->data.setSortKey(edge.getSortKey());
                forwardQueue_.update(*it);
            }
            else
            {
                auto element = forwardQueue_.insert(edge);
                edge.getParent()->addToForwardQueueOutgoingLookup(element);
                edge.getChild()->addToForwardQueueIncomingLookup(element);
            }
        }

        std::shared_ptr<ompl::geometric::PathGeometric>
        AITstar::getPathToVertex(const std::shared_ptr<aitstar::Vertex> &vertex) const
        {
            // Create the reverse path by following the parents to the start.
            std::vector<std::shared_ptr<aitstar::Vertex>> reversePath;
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

        std::array<ompl::base::Cost, 3u> AITstar::computeSortKey(const std::shared_ptr<aitstar::Vertex> &parent,
                                                                 const std::shared_ptr<aitstar::Vertex> &child) const
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

        std::array<ompl::base::Cost, 2u> AITstar::computeSortKey(const std::shared_ptr<aitstar::Vertex> &vertex) const
        {
            // LPA* sort key is [min(g(x), v(x)) + h(x); min(g(x), v(x))].
            return {objective_->combineCosts(objective_->betterCost(vertex->getCostToComeFromGoal(),
                                                                    vertex->getExpandedCostToComeFromGoal()),
                                             computeCostToGoToStartHeuristic(vertex)),
                    objective_->betterCost(vertex->getCostToComeFromGoal(), vertex->getExpandedCostToComeFromGoal())};
        }

        std::vector<aitstar::Edge> AITstar::getOutgoingEdges(const std::shared_ptr<aitstar::Vertex> &vertex) const
        {
            // Prepare the return variable.
            std::vector<aitstar::Edge> outgoingEdges;

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

        bool AITstar::haveAllVerticesBeenProcessed(const std::vector<aitstar::Edge> &edges) const
        {
            for (const auto &edge : edges)
            {
                if (!haveAllVerticesBeenProcessed(edge))
                {
                    return false;
                }
            }

            return true;
        }

        bool AITstar::haveAllVerticesBeenProcessed(const aitstar::Edge &edge) const
        {
            return edge.getParent()->hasBeenExpandedDuringCurrentReverseSearch() &&
                   edge.getChild()->hasBeenExpandedDuringCurrentReverseSearch();
        }

        void AITstar::updateExactSolution()
        {
            // Check if any of the goals have a cost to come less than the current solution cost.
            for (const auto &goal : graph_.getGoalVertices())
            {
                // We need to check whether the cost is better, or whether someone has removed the exact solution from
                // the problem definition.
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

        void AITstar::updateApproximateSolution(const std::shared_ptr<aitstar::Vertex> &vertex)
        {
            assert(trackApproximateSolutions_);
            if (vertex->hasForwardParent() || graph_.isStart(vertex))
            {
                auto costToGoal = computeCostToGoToGoal(vertex);

                // We need to check whether this is better than the current approximate solution or whether someone has
                // removed all approximate solutions from the problem definition.
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

        ompl::base::Cost AITstar::computeCostToGoToStartHeuristic(const std::shared_ptr<aitstar::Vertex> &vertex) const
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

        ompl::base::Cost AITstar::computeCostToGoToGoalHeuristic(const std::shared_ptr<aitstar::Vertex> &vertex) const
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

        ompl::base::Cost AITstar::computeCostToGoToGoal(const std::shared_ptr<aitstar::Vertex> &vertex) const
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

        void AITstar::invalidateCostToComeFromGoalOfReverseBranch(const std::shared_ptr<aitstar::Vertex> &vertex)
        {
            vertex->unregisterExpansionDuringReverseSearch();
            // Update the cost of all reverse children and remove from open.
            for (const auto &child : vertex->getReverseChildren())
            {
                invalidateCostToComeFromGoalOfReverseBranch(child);
                child->setCostToComeFromGoal(objective_->infiniteCost());
                child->setExpandedCostToComeFromGoal(objective_->infiniteCost());
                auto reverseQueuePointer = child->getReverseQueuePointer();
                if (reverseQueuePointer)
                {
                    reverseQueue_.remove(reverseQueuePointer);
                    child->resetReverseQueuePointer();
                }
            }
        }

    }  // namespace geometric
}  // namespace ompl

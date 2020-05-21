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

#pragma once

#include <algorithm>
#include <memory>

#include "ompl/base/Planner.h"
#include "ompl/datastructures/BinaryHeap.h"
#include "ompl/geometric/planners/aitstar/datastructures/Edge.h"
#include "ompl/geometric/planners/aitstar/datastructures/ImplicitGraph.h"
#include "ompl/geometric/planners/aitstar/datastructures/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        /**
        @anchor gAITstar

        \ref gAITstar "AIT*" (Adaptively Informed Trees) is an almost-surely asymptotically optimal path planner.
        It aims to find an initial solution quickly and asymptotically converge to the globally optimal solution.

        AIT* views the planning problem as the two subproblems of approximation and search, and approximates the free
        state space with an increasingly dense, edge-implicit random geometric graph (RGG), similar to BIT* and ABIT*.

        AIT* uses an asymmetric bidirectional search that simultaneously estimates and exploits an accurate, adaptive
        heuristic that is specific to each problem instance. The reverse search (goal to start) does not perform
        collision detection on the edges and estimates cost-to-go values for all states in the RGG that are processed
        with the forward search (start to goal) which does perform collision detection. Because AIT* uses LPA* as the
        reverse search algorithm, the heuristic is admissible in the context of the current RGG and can efficiently be
        updated when the forward search detects a collision on an edge that is in the reverse search tree (as this
        indicates a flawed heuristic).

        This implementation of AIT* can solve problems with multiple start and goal states and supports adding start and
        goal states while the planner is running.

        @par Associated publication:

        M. P. Strub, J. D. Gammell. “Adaptively Informed Trees (AIT*): Fast Asymptotically Optimal Path Planning through
        Adaptive Heuristics” in Proceedings of the IEEE international conference on robotics and automation (ICRA),
        Paris, France, 31 May – 4 Jun. 2020.

        DOI: <a href="https://arxiv.org/abs/2002.06599">arXiv:2002.06589</a>
        Video 1: <a href="https://youtu.be/twM723QM9TQ">ICRA submission video</a>.
        Video 2: <a href="https://youtu.be/1h7ugF9F6VM">ICRA presentation video</a>
        */

        /** \brief Adaptively Informed Trees (AIT*) */
        class AITstar : public ompl::base::Planner
        {
        public:
            /** \brief Constructs a AIT*. */
            AITstar(const ompl::base::SpaceInformationPtr &spaceInformation);

            /** \brief Destructs a AIT*. */
            ~AITstar() = default;

            /** \brief Additional setup that can only be done once a problem definition is set. */
            void setup() override;

            /** \brief Clears the algorithm's internal state. */
            void clear() override;

            /** \brief Solves a motion planning problem. */
            ompl::base::PlannerStatus
            solve(const ompl::base::PlannerTerminationCondition &terminationCondition) override;

            /** \brief Get the cost of the incumbent solution. */
            ompl::base::Cost bestCost() const;

            /** \brief Get the planner data. */
            void getPlannerData(base::PlannerData &data) const override;

            /** \brief Set the batch size. */
            void setBatchSize(std::size_t batchSize);

            /** \brief Get the batch size. */
            std::size_t getBatchSize() const;

            /** \brief Set the rewire factor of the RGG graph. */
            void setRewireFactor(double rewireFactor);

            /** \brief Get the rewire factor of the RGG graph. */
            double getRewireFactor() const;

            /** \brief Set whether to track approximate vertices. */
            void trackApproximateSolutions(bool track);

            /** \brief Set whether to track approximate vertices. */
            bool areApproximateSolutionsTracked() const;

            /** \brief Updates the best approximate goal vertex. */
            void updateBestApproximateGoalVertex();

            /** \brief Set whether pruning is enabled or not. */
            void enablePruning(bool prune);

            /** \brief Set whether pruning is enabled or not. */
            bool isPruningEnabled() const;

            /** \brief Enable LPA* repair of backward search. */
            void setRepairBackwardSearch(bool setRepairBackwardSearch);

            /** \brief Stop when finding an improved solution. */
            void setStopOnFindingInitialSolution(bool stopOnFindingInitialSolution);

            /** \brief Get the edge queue. */
            std::vector<aitstar::Edge> getEdgesInQueue() const;

            /** \brief Get the vertex queue. */
            std::vector<std::shared_ptr<aitstar::Vertex>> getVerticesInQueue() const;

            /** \brief Get the next edge in the queue. */
            aitstar::Edge getNextEdgeInQueue() const;

            /** \brief Get the next vertex in the queue. */
            std::shared_ptr<aitstar::Vertex> getNextVertexInQueue() const;

            /** \brief Get the vertices in the backward search tree. */
            std::vector<std::shared_ptr<aitstar::Vertex>> getVerticesInBackwardSearchTree() const;

        private:
            /** \brief Performs one iteration of AIT*. */
            void iterate();

            /** \brief Performs one forward search iterations. */
            void performForwardSearchIteration();

            /** \brief Performs one backward search iterations. */
            void performBackwardSearchIteration();

            /** \brief Updates a vertex in the backward search queue (LPA* update). */
            void backwardSearchUpdateVertex(const std::shared_ptr<aitstar::Vertex> &vertex);

            /** \brief Inserts or updates a vertex in the backward queue. */
            void insertOrUpdateInForwardQueue(const aitstar::Edge &edge);

            /** \brief Inserts or updates a vertex in the backward queue. */
            void insertOrUpdateInBackwardQueue(const std::shared_ptr<aitstar::Vertex> &vertex);

            /** \brief Rebuilds the forward queue. */
            void rebuildForwardQueue();

            /** \brief Rebuilds the forward queue. */
            void rebuildBackwardQueue();

            /** \brief Returns a vector of states from the argument to a start. */
            std::vector<std::shared_ptr<aitstar::Vertex>>
            getReversePath(const std::shared_ptr<aitstar::Vertex> &vertex) const;

            /** \brief Computes the sort key of an edge. */
            std::array<ompl::base::Cost, 3u> computeSortKey(const std::shared_ptr<aitstar::Vertex> &parent,
                                                            const std::shared_ptr<aitstar::Vertex> &child) const;

            /** \brief Computes the sort key of a vertex. */
            std::array<ompl::base::Cost, 2u> computeSortKey(const std::shared_ptr<aitstar::Vertex> &vertex) const;

            /** \brief Inserts the outgoing edges of a vertex into the forward edge queue. */
            void insertOutgoingEdges(const std::shared_ptr<aitstar::Vertex> &vertex);

            /** \brief Get all outgoing edges of a vertex. */
            std::vector<aitstar::Edge> getOutgoingEdges(const std::shared_ptr<aitstar::Vertex> &vertex) const;

            /** \brief Check whether all vertices of a set of edges have been processed by the reverse queue. */
            bool haveAllVerticesBeenProcessed(const std::vector<aitstar::Edge> &edges) const;

            /** \brief Check whether the parent and child vertices of an edge have been processed by the reverse queue.
             */
            bool haveAllVerticesBeenProcessed(const aitstar::Edge &edges) const;

            /** \brief Checks whether the cost to come of a goal vertex has been updated and updates the solution if so.
             */
            void updateSolution();

            /** \brief Returns the best cost-to-go-heuristic to any start in the graph. */
            ompl::base::Cost computeCostToGoToStartHeuristic(const std::shared_ptr<aitstar::Vertex> &vertex) const;

            /** \brief Returns the best cost-to-go-heuristic to any goal in the graph. */
            ompl::base::Cost computeCostToGoToGoalHeuristic(const std::shared_ptr<aitstar::Vertex> &vertex) const;

            /** \brief Returns the best cost to any goal in the graph. */
            ompl::base::Cost computeCostToGoToGoal(const std::shared_ptr<aitstar::Vertex> &vertex) const;

            /** \brief Returns the best cost to come form the goal of any start. */
            ompl::base::Cost computeBestCostToComeFromGoalOfAnyStart() const;

            /** \brief Sets the cost to come from to goal of the backward search to infinity for the whole branch.
             */
            void invalidateCostToComeFromGoalOfBackwardBranch(const std::shared_ptr<aitstar::Vertex> &vertex);

            /** \brief The increasingly dense sampling-based approximation. */
            aitstar::ImplicitGraph graph_;

            /** \brief The forward queue. */
            using EdgeQueue =
                ompl::BinaryHeap<aitstar::Edge, std::function<bool(const aitstar::Edge &, const aitstar::Edge &)>>;
            std::unique_ptr<EdgeQueue> forwardQueue_;

            /** \brief The backward queue. */
            using KeyVertexPair = std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<aitstar::Vertex>>;
            using VertexQueue =
                ompl::BinaryHeap<KeyVertexPair, std::function<bool(const KeyVertexPair &, const KeyVertexPair &)>>;
            std::unique_ptr<VertexQueue> backwardQueue_;

            /** \brief The id of the current forward search .*/
            std::shared_ptr<std::size_t> forwardSearchId_;

            /** \brief The id of the current backward search .*/
            std::shared_ptr<std::size_t> backwardSearchId_;

            /** \brief The cost of the incumbent solution. */
            std::shared_ptr<ompl::base::Cost> solutionCost_;

            /** \brief The vertex that is closest to the goal (in cost space). */
            ompl::base::Cost approximateSolutionCost_{};

            /** \brief The edges to be inserted in the forward queue. */
            std::vector<aitstar::Edge> edgesToBeInserted_{};

            /** \brief The number of iterations that have been performed. */
            std::size_t numIterations_{0u};

            /** \brief The number of samples per batch. */
            std::size_t batchSize_{100u};

            /** \brief Flag whether to perform a backward search iteration on the next iteration. */
            bool performBackwardSearchIteration_{true};

            /** \brief Flag whether the forward search has been started on the batch. */
            bool isForwardSearchStartedOnBatch_{false};

            /** \brief Flag whether the forward queue needs to be rebuilt. */
            bool forwardQueueMustBeRebuilt_{false};

            /** \brief The option that specifies whether to repair the backward search when detecting a collision. */
            bool repairBackwardSearch_{true};

            /** \brief The option that specifies whether to track approximate solutions. */
            bool trackApproximateSolutions_{false};

            /** \brief The option that specifies whether to prune the graph of useless samples. */
            bool isPruningEnabled_{true};

            /** \brief Whether to stop on finding an improved solution. */
            bool stopOnFindingInitialSolution_{false};

            /** \brief Syntactic helper to get at the optimization objective of the planner base class. */
            ompl::base::OptimizationObjectivePtr objective_;

            /** \brief Syntactic helper to get at the motion validator of the planner base class. */
            ompl::base::MotionValidatorPtr motionValidator_;

            /** \brief The number of edge collision checks performed. */
            std::size_t numEdgeCollisionChecks_{0u};
        };
    }  // namespace geometric
}  // namespace ompl

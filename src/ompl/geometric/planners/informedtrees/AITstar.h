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

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_AITSTAR_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_AITSTAR_

#include <algorithm>
#include <memory>

#include "ompl/base/Planner.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/planners/informedtrees/aitstar/Edge.h"
#include "ompl/geometric/planners/informedtrees/aitstar/ImplicitGraph.h"
#include "ompl/geometric/planners/informedtrees/aitstar/Vertex.h"
#include "ompl/geometric/planners/informedtrees/aitstar/Queuetypes.h"

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
        goal states while the planner is running. One can also turn off repairing the reverse search tree upon collision
        detection, which might be beneficial for problems in which collision detection is computationally inexpensive.

        @par Associated publications:

        M. P. Strub, J. D. Gammell. “Adaptively Informed Trees (AIT*): Fast Asymptotically Optimal Path Planning through
        Adaptive Heuristics” in Proceedings of the IEEE international conference on robotics and automation (ICRA),
        Paris, France, 31 May – 4 Jun. 2020.

        arXiv: <a href="https://arxiv.org/abs/2002.06599">arXiv:2002.06589</a>
        DOI: <a href="https://doi.org/10.1109/ICRA40945.2020.9197338">ICRA40945.2020.9197338</a>
        Video 1: <a href="https://youtu.be/twM723QM9TQ">ICRA submission video</a>.
        Video 2: <a href="https://youtu.be/1h7ugF9F6VM">ICRA presentation video</a>

        M. P. Strub, J. D. Gammell. “AIT* and EIT*: Asymmetric bidirectional sampling-based path planning”
        The International Journal of Robotics Research (IJRR), in revision, 2021.

        arXiv: <a href="https://arxiv.org/abs/2111.01877">arXiv:2111.01877</a>
        Video 1: <a href="https://youtu.be/aC2xBwOcDoo">IJRR trailer</a>
        Video 2: <a href="https://youtu.be/ux4-Mui44DE">IJRR extension 1</a>

        */

        /** \brief Adaptively Informed Trees (AIT*) */
        class AITstar : public ompl::base::Planner
        {
        public:
            /** \brief Constructs a AIT*. */
            explicit AITstar(const ompl::base::SpaceInformationPtr &spaceInformation);

            /** \brief Destructs a AIT*. */
            ~AITstar() = default;

            /** \brief Additional setup that can only be done once a problem definition is set. */
            void setup() override;

            /** \brief Checks whether the planner is successfully setup. */
            ompl::base::PlannerStatus::StatusType ensureSetup();

            /** \brief Checks whether the problem is successfully setup. */
            ompl::base::PlannerStatus::StatusType
            ensureStartAndGoalStates(const ompl::base::PlannerTerminationCondition &terminationCondition);

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

            /** \brief Set whether to track approximate solutions. */
            void trackApproximateSolutions(bool track);

            /** \brief Get whether approximate solutions are tracked. */
            bool areApproximateSolutionsTracked() const;

            /** \brief Set whether pruning is enabled or not. */
            void enablePruning(bool prune);

            /** \brief Get whether pruning is enabled or not. */
            bool isPruningEnabled() const;

            /** \brief Set whether to use a k-nearest RGG connection model. If false, AIT* uses an r-disc model. */
            void setUseKNearest(bool useKNearest);

            /** \brief Get whether to use a k-nearest RGG connection model. If false, AIT* uses an r-disc model. */
            bool getUseKNearest() const;

            /** \brief Set the maximum number of goals AIT* will sample from sampleable goal regions. */
            void setMaxNumberOfGoals(unsigned int numberOfGoals);

            /** \brief Get the maximum number of goals AIT* will sample from sampleable goal regions. */
            unsigned int getMaxNumberOfGoals() const;

            /** \brief Get the edge queue. */
            std::vector<aitstar::Edge> getEdgesInQueue() const;

            /** \brief Get the vertex queue. */
            std::vector<std::shared_ptr<aitstar::Vertex>> getVerticesInQueue() const;

            /** \brief Get the next edge in the queue. */
            aitstar::Edge getNextEdgeInQueue() const;

            /** \brief Get the next vertex in the queue. */
            std::shared_ptr<aitstar::Vertex> getNextVertexInQueue() const;

            /** \brief Get the vertices in the reverse search tree. */
            std::vector<std::shared_ptr<aitstar::Vertex>> getVerticesInReverseSearchTree() const;

        private:
            /** \brief Performs one iteration of AIT*. */
            void iterate(const ompl::base::PlannerTerminationCondition &terminationCondition);

            /** \brief Performs one forward search iterations. */
            void iterateForwardSearch();

            /** \brief Performs one reverse search iterations. */
            void iterateReverseSearch();

            /** \brief Updates a vertex in the reverse search queue (LPA* update). */
            void updateReverseSearchVertex(const std::shared_ptr<aitstar::Vertex> &vertex);

            /** \brief Updates all neighbors of a reverse search vertex. */
            void updateReverseSearchNeighbors(const std::shared_ptr<aitstar::Vertex> &vertex);

            /** \brief Inserts or updates an edge in the forward queue. */
            void insertOrUpdateInForwardQueue(const aitstar::Edge &edge);

            /** \brief Inserts or updates edges in the forward queue. */
            void insertOrUpdateInForwardQueue(const std::vector<aitstar::Edge> &edges);

            /** \brief Inserts or updates a vertex in the reverse queue. */
            void insertOrUpdateInReverseQueue(const std::shared_ptr<aitstar::Vertex> &vertex);

            /** \brief Rebuilds the forward queue. */
            void rebuildForwardQueue();

            /** \brief Rebuilds the forward queue. */
            void rebuildReverseQueue();

            /** \brief Clears the forward queue, resetting the lookups of the vertices in it. */
            void clearForwardQueue();

            /** \brief Clears the reverse queue, resetting the lookups of the vertices in it. */
            void clearReverseQueue();

            /** \brief Prints a message using OMPL_INFORM to let the user know that AIT* found a new solution. */
            void informAboutNewSolution() const;

            /** \brief Prints a message using OMPL_INFORM to let the user know of the planner status. */
            void informAboutPlannerStatus(ompl::base::PlannerStatus::StatusType status) const;

            /** \brief Inserts the goal vertices of the graph into the reverse search queue. */
            void insertGoalVerticesInReverseQueue();

            /** \brief Insert start vertices in the forward queue. */
            void expandStartVerticesIntoForwardQueue();

            /** \brief Check whether the reverse search must be continued. */
            bool continueReverseSearch() const;

            /** \brief Check whether the forward search must be continued. */
            bool continueForwardSearch();

            /** \brief Returns the path a start to the argument. */
            std::shared_ptr<ompl::geometric::PathGeometric>
            getPathToVertex(const std::shared_ptr<aitstar::Vertex> &vertex) const;

            /** \brief Computes the sort key of an edge. */
            std::array<ompl::base::Cost, 3u> computeSortKey(const std::shared_ptr<aitstar::Vertex> &parent,
                                                            const std::shared_ptr<aitstar::Vertex> &child) const;

            /** \brief Computes the sort key of a vertex. */
            std::array<ompl::base::Cost, 2u> computeSortKey(const std::shared_ptr<aitstar::Vertex> &vertex) const;

            /** \brief Inserts the outgoing edges of a vertex into the forward edge queue. */
            void insertOutgoingEdges(const std::shared_ptr<aitstar::Vertex> &vertex);

            /** \brief Get all outgoing edges of a vertex. */
            std::vector<aitstar::Edge> getOutgoingEdges(const std::shared_ptr<aitstar::Vertex> &vertex) const;

            /** \brief Checks whether the cost to come of a goal vertex has been updated and updates the solution if so.
             */
            void updateExactSolution();

            /** \brief Checks whether the input vertex is the new best approximate solution and updates the solution in
             * the problem definition if so. **/
            void updateApproximateSolution(const std::shared_ptr<aitstar::Vertex> &vertex);

            /** \brief Checks which vertex is the best approximate solution. */
            void updateApproximateSolution();

            /** \brief Updates the exact solution and if AIT* track approximate solutions, it updates it as well. */
            ompl::base::PlannerStatus::StatusType updateSolution();

            /** \brief Updates the exact solution and if AIT* track approximate solutions, it updates it as well. */
            ompl::base::PlannerStatus::StatusType updateSolution(const std::shared_ptr<aitstar::Vertex> &vertex);

            /** \brief Returns the best cost-to-go-heuristic to any start in the graph. */
            ompl::base::Cost computeCostToGoToStartHeuristic(const std::shared_ptr<aitstar::Vertex> &vertex) const;

            /** \brief Returns the best cost-to-go-heuristic to any goal in the graph. */
            ompl::base::Cost computeCostToGoToGoalHeuristic(const std::shared_ptr<aitstar::Vertex> &vertex) const;

            /** \brief Returns the best cost to go to any goal in the graph. */
            ompl::base::Cost computeCostToGoToGoal(const std::shared_ptr<aitstar::Vertex> &vertex) const;

            /** \brief Returns the best cost to come form the goal of any start. */
            ompl::base::Cost computeBestCostToComeFromGoalOfAnyStart() const;

            /** \brief Counts the number of vertices in the forward tree. */
            std::size_t countNumVerticesInForwardTree() const;

            /** \brief Counts the number of vertices in the reverse tree. */
            std::size_t countNumVerticesInReverseTree() const;

            /** \brief Sets the cost to come from to goal of the reverse search to infinity for the whole branch.
             */
            void invalidateCostToComeFromGoalOfReverseBranch(const std::shared_ptr<aitstar::Vertex> &vertex);

            /** \brief The cost of the incumbent solution. */
            ompl::base::Cost solutionCost_;

            /** \brief The cost to come to the vertex that is closest to the goal (in cost space). */
            ompl::base::Cost approximateSolutionCost_{};

            /** \brief The cost to go to the goal from the current best approximate solution. */
            ompl::base::Cost approximateSolutionCostToGoal_{};

            /** \brief The increasingly dense sampling-based approximation. */
            aitstar::ImplicitGraph graph_;

            /** \brief The forward queue. */
            aitstar::EdgeQueue forwardQueue_;

            /** \brief The reverse queue. */
            aitstar::VertexQueue reverseQueue_;

            /** \brief Lexicographically compares the keys of two edges. */
            bool isEdgeBetter(const aitstar::Edge &lhs, const aitstar::Edge &rhs) const;

            /** \brief Lexicographically compares the keys of two vertices. */
            bool isVertexBetter(const aitstar::KeyVertexPair &lhs, const aitstar::KeyVertexPair &rhs) const;

            /** \brief the number of edges in the forward queue with inconsistent target states. */
            std::size_t numInconsistentOrUnconnectedTargets_{0u};

            /** \brief The number of iterations that have been performed. */
            std::size_t numIterations_{0u};

            /** \brief The number of samples per batch. */
            std::size_t batchSize_{100u};

            /** \brief The option that specifies whether to track approximate solutions. */
            bool trackApproximateSolutions_{true};

            /** \brief The option that specifies whether to prune the graph of useless samples. */
            bool isPruningEnabled_{true};

            /** \brief Syntactic helper to get at the optimization objective of the planner base class. */
            ompl::base::OptimizationObjectivePtr objective_;

            /** \brief Syntactic helper to get at the motion validator of the planner base class. */
            ompl::base::MotionValidatorPtr motionValidator_;

            /** \brief The number of processed edges. */
            std::size_t numProcessedEdges_{0u};

            /** \brief The number of edge collision checks performed. */
            std::size_t numEdgeCollisionChecks_{0u};
        };
    }  // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_AITSTAR

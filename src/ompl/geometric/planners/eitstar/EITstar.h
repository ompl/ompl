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

#ifndef OMPL_GEOMETRIC_PLANNERS_EITSTAR_EITSTAR_
#define OMPL_GEOMETRIC_PLANNERS_EITSTAR_EITSTAR_

#include <memory>

#include "ompl/base/Cost.h"
#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"

#include "ompl/geometric/planners/eitstar/Direction.h"
#include "ompl/geometric/planners/eitstar/RandomGeometricGraph.h"
#include "ompl/geometric/planners/eitstar/ForwardQueue.h"
#include "ompl/geometric/planners/eitstar/ReverseQueue.h"

namespace ompl
{
    namespace geometric
    {
        class EITstar : public ompl::base::Planner
        {
        public:
            /** \brief Constructs an algorithm using the provided space information */
            EITstar(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo);

            /** \brief Destructs the algorithm. */
            ~EITstar();

            /** \brief Setup the parts of the planner that rely on the problem definition being set. */
            void setup() override;

            /** \brief Solves the provided motion planning problem. */
            ompl::base::PlannerStatus
            solve(const ompl::base::PlannerTerminationCondition &terminationCondition) override;

            /** \brief Gets the cost of the current best solution. */
            ompl::base::Cost bestCost() const;

            /** \brief Sets the number of samples per batch. */
            void setNumSamplesPerBatch(std::size_t numSamples);

            /** \brief Sets the initial number of collision checks on the reverse search. */
            void setInitialNumberOfSparseCollisionChecks(std::size_t numChecks);

            /** \brief Sets the radius factor. */
            void setRadiusFactor(double factor);

            /** \brief Sets the radius factor. */
            void setSuboptimalityFactor(double factor);

            /** \brief Sets the option whether to increse collision detection on the reverse search tree when the
             * forward search detects a collision. */
            void enableCollisionDetectionInReverseSearch(bool enable);

            /** \brief Whether increasingly dense collision detection in the reverse search is enabled. */
            bool isCollisionDetectionInReverseSearchEnabled() const;

            /** \brief Set whether pruning is enabled or not. */
            void enablePruning(bool prune);

            /** \brief Get whether pruning is enabled or not. */
            bool isPruningEnabled() const;

            /** \brief Returns a copy of the forward queue. */
            std::vector<eitstar::Edge> getForwardQueue() const;

            /** \brief Returns a copy of the reverse queue. */
            std::vector<eitstar::Edge> getReverseQueue() const;

            /** \brief Returns the edges in the reverse tree. */
            std::vector<eitstar::Edge> getReverseTree() const;

            /** \brief Returns the next edge in the forward queue. */
            eitstar::Edge getNextForwardEdge() const;

            /** \brief Returns the next edge in the reverse queue. */
            eitstar::Edge getNextReverseEdge() const;

            /** \brief Checks whether the vertex is a start state. */
            bool isStart(const std::shared_ptr<eitstar::State> &state) const;

            /** \brief Checks whether the vertex is a goal State. */
            bool isGoal(const std::shared_ptr<eitstar::State> &state) const;

            /** \brief Get the planner data. */
            void getPlannerData(base::PlannerData &data) const override;

        private:
            /** \brief The different phases the algorithm can be in. */
            enum class Phase
            {
                FORWARD_SEARCH,
                REVERSE_SEARCH,
                IMPROVE_APPROXIMATION
            };

            /** \brief The phase the algorithm is in. */
            Phase phase_{Phase::REVERSE_SEARCH};

            /** \brief Performs one iteration. */
            void iterate();

            /** \brief Perform one reverse iteration. */
            void reverseIterate();

            /** \brief Perform one forward iteration. */
            void forwardIterate();

            /** \brief Improves the approximation by sampling more states. */
            void improveApproximation();

            /** \brief Updates the solution. */
            void updateSolution(const std::shared_ptr<eitstar::State> &goalState);

            /** \brief Increases the collision detection resolution and restart reverse search. */
            void increaseSparseCollisionDetectionResolutionAndRestartReverseSearch();

            /** \brief Rewire reverse search tree locally. Returns [ bestParent, bestCost, bestEdgeCost ].
             * Note that bestParent == nullptr if no parent is found. */
            std::tuple<std::shared_ptr<eitstar::State>, ompl::base::Cost, ompl::base::Cost>
            getBestParentInReverseTree(const std::shared_ptr<eitstar::State> &state) const;

            /** \brief Expands the input state, creating forward edges. */
            std::vector<eitstar::Edge> expand(const std::shared_ptr<eitstar::State> &state) const;

            /** \brief Expands the forward roots that are in the reverse search tree. */
            std::vector<eitstar::Edge> expandForwardRootsInReverseTree() const;

            /** \brief Returns whether the vertex has been closed during the current search. */
            bool isClosed(const std::shared_ptr<eitstar::Vertex> &vertex) const;

            /** \brief Returns whether the edge can improve the reverse path. */
            bool doesImproveReversePath(const eitstar::Edge &edge) const;

            /** \brief Returns whether the edge can improve the reverse tree. */
            bool doesImproveReverseTree(const eitstar::Edge &edge, const ompl::base::Cost &admissibleEdgeCost) const;

            /** \brief Returns whether the edge can improve the forward path. */
            bool couldImproveForwardPath(const eitstar::Edge &edge) const;

            /** \brief Returns whether the edge can improve the forward tree. */
            bool couldImproveForwardTree(const eitstar::Edge &edge) const;

            /** \brief Returns whether the edge does improve the forward path. */
            bool doesImproveForwardPath(const eitstar::Edge &edge, const ompl::base::Cost &trueEdgeCost) const;

            /** \brief Returns whether the edge does improve the forward tree. */
            bool doesImproveForwardTree(const eitstar::Edge &edge, const ompl::base::Cost &trueEdgeCost) const;

            /** \brief Returns whether the edge is valid. */
            bool isValid(const eitstar::Edge &edge) const;

            /** \brief Returns whether the edge could be valid. */
            bool couldBeValid(const eitstar::Edge &edge) const;

            /** \brief Expands and inserts the reverse roots into the reverse queue. */
            void expandReverseRootsIntoReverseQueue();

            /** \brief Returns whether any forward root is in the reverse search tree. */
            bool isAnyForwardRootInReverseTree() const;

            /** \brief Returns whether any reverse root is in the forward search tree. */
            bool isAnyReverseRootInForwardTree() const;

            /** \brief Returns whether all vertices have been processed by the reverse search. */
            bool canBeInsertedInForwardQueue(const std::vector<eitstar::Edge> &edges) const;

            /** \brief The sampling-based approximation of the state space. */
            eitstar::RandomGeometricGraph graph_;

            /** \brief The number of states added when the approximation is updated. */
            std::size_t numSamplesPerBatch_{100u};

            /** \brief The current suboptimality factor of the forward search. */
            double suboptimalityFactor_{std::numeric_limits<float>::infinity()};

            /** \brief The option that specifies whether sparse collision detection on the reverse search tree is
             * enabled. */
            bool isCollisionDetectionInReverseTreeEnabled_{true};

            /** \brief The number of sparse collision detections on level 0. */
            std::size_t initialNumSparseCollisionChecks_{1u};

            /** \brief The number of sparse collision detections performed on the reverse search on this level. */
            std::size_t numSparseCollisionChecksCurrentLevel_{1u};

            /** \brief The number of sparse collision detections performed on the reverse search on the previous level. */
            std::size_t numSparseCollisionChecksPreviousLevel_{0u};

            /** \brief Whether pruning is enabled. */
            bool isPruningEnabled_{true};

            /** \brief The edge cache that enables the just-in-time reverse search. */
            std::vector<eitstar::Edge> jitSearchEdgeCache_{};

            /** \brief The state used to do sparse collision detection with. */
            ompl::base::State *detectionState_;

            /** \brief The roots of the forward search tree (forest). */
            std::vector<std::shared_ptr<eitstar::Vertex>> forwardRoots_;

            /** \brief The roots of the reverse search tree (forest). */
            std::vector<std::shared_ptr<eitstar::Vertex>> reverseRoots_;

            /** \brief The forward queue. */
            std::unique_ptr<eitstar::ForwardQueue> forwardQueue_;

            /** \brief The reverse queue. */
            std::unique_ptr<eitstar::ReverseQueue> reverseQueue_;

            /** \brief The current iteration. */
            std::size_t iteration_{0u};

            /** \brief The tag of the current search. */
            std::size_t searchTag_{1u};

            /** \brief The tag of the current batch. */
            std::size_t startExpansionGraphTag_{0u};

            /** \brief An alias with a more expressive name to the problem of the base class. */
            std::shared_ptr<ompl::base::ProblemDefinition> &problem_ = ompl::base::Planner::pdef_;

            /** \brief An alias with a more expressive name to the space information of the base class. */
            std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo_ = ompl::base::Planner::si_;

            /** \brief A direct pointer to the state space. */
            std::shared_ptr<ompl::base::StateSpace> space_;

            /** \brief Direct access to the optimization objective of the problem. */
            std::shared_ptr<ompl::base::OptimizationObjective> objective_;

            /** \brief Direct access to the motion validator of the state space. */
            std::shared_ptr<ompl::base::MotionValidator> motionValidator_;

            /** \brief The cost of the incumbent solution. */
            ompl::base::Cost solutionCost_{std::numeric_limits<double>::signaling_NaN()};

            /** \brief The cost of the best reverse path. */
            ompl::base::Cost reverseCost_;
        };

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_EITSTAR_EITSTAR_

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

#ifndef OMPL_GEOMETRIC_PLANNERS_AIBITSTAR_AIBITSTAR_
#define OMPL_GEOMETRIC_PLANNERS_AIBITSTAR_AIBITSTAR_

#include <memory>

#include "ompl/base/Cost.h"
#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"

#include "ompl/geometric/planners/aeitstar/Direction.h"
#include "ompl/geometric/planners/aeitstar/RandomGeometricGraph.h"
#include "ompl/geometric/planners/aeitstar/ForwardQueue.h"
#include "ompl/geometric/planners/aeitstar/ReverseQueue.h"

namespace ompl
{
    namespace geometric
    {
        class AEITstar : public ompl::base::Planner
        {
        public:
            /** \brief Constructs an algorithm using the provided space information */
            AEITstar(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo);

            /** \brief Destructs the algorithm. */
            ~AEITstar();

            /** \brief Setup the parts of the planner that rely on the problem definition being set. */
            void setup() override;

            /** \brief Solves the provided motion planning problem. */
            ompl::base::PlannerStatus
            solve(const ompl::base::PlannerTerminationCondition &terminationCondition) override;

            /** \brief Gets the cost of the current best solution. */
            ompl::base::Cost bestCost() const;

            /** \brief Sets the number of samples per batch. */
            void setNumSamplesPerBatch(std::size_t numSamples);

            /** \brief Sets the radius factor. */
            void setRadiusFactor(double factor);

            /** \brief Sets the radius factor. */
            void setRepairFactor(double factor);

            /** \brief Sets the radius factor. */
            void setSuboptimalityFactor(double factor);

            /** \brief Sets the option whether to repair the reverse search tree when the forward search detects a
             * collision. */
            void enableRepairingReverseTree(bool enable);

            /** \brief Sets the option whether to repair the reverse search tree when the forward search detects a
             * collision. */
            void enableCollisionDetectionInReverseSearch(bool enable);

            /** \brief Returns a copy of the forward queue. */
            std::vector<aeitstar::Edge> getForwardQueue() const;

            /** \brief Returns a copy of the reverse queue. */
            std::vector<aeitstar::Edge> getReverseQueue() const;

            /** \brief Returns the edges in the reverse tree. */
            std::vector<aeitstar::Edge> getReverseTree() const;

            /** \brief Returns the next edge in the forward queue. */
            aeitstar::Edge getNextForwardEdge() const;

            /** \brief Returns the next edge in the reverse queue. */
            aeitstar::Edge getNextReverseEdge() const;

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

            /** \brief Updates the solution. */
            void updateSolution();

            /** \brief Repairs the reverse search tree upon finding an invalid edge. */
            void repairReverseSearchTree(const aeitstar::Edge &invalidEdge,
                                         std::shared_ptr<aeitstar::State> &invalidatedState);

            /** \brief Increases the collision detection resolution and restart reverse search. */
            void increaseSparseCollisionDetectionResolutionAndRestartReverseSearch();

            /** \brief Rewire reverse search tree locally. Returns [ bestParent, bestCost, bestEdgeCost ].
             * Note that bestParent == nullptr if no parent is found. */
            std::tuple<std::shared_ptr<aeitstar::State>, ompl::base::Cost, ompl::base::Cost>
            getBestParentInReverseTree(const std::shared_ptr<aeitstar::State> &state) const;

            /** \brief Expands the input state, creating forward edges. */
            std::vector<aeitstar::Edge> expand(const std::shared_ptr<aeitstar::State> &state) const;

            /** \brief Returns whether the vertex has been closed during the current search. */
            bool isClosed(const std::shared_ptr<aeitstar::Vertex> &vertex) const;

            /** \brief Returns whether the edge can improve the reverse path. */
            bool doesImproveReversePath(const aeitstar::Edge &edge) const;

            /** \brief Returns whether the edge can improve the reverse tree. */
            bool doesImproveReverseTree(const aeitstar::Edge &edge) const;

            /** \brief Returns whether the edge can improve the forward path. */
            bool couldImproveForwardPath(const aeitstar::Edge &edge) const;

            /** \brief Returns whether the edge can improve the forward tree. */
            bool couldImproveForwardTree(const aeitstar::Edge &edge) const;

            /** \brief Returns whether the edge does improve the forward path. */
            bool doesImproveForwardPath(const aeitstar::Edge &edge, const ompl::base::Cost &trueEdgeCost) const;

            /** \brief Returns whether the edge does improve the forward tree. */
            bool doesImproveForwardTree(const aeitstar::Edge &edge, const ompl::base::Cost &trueEdgeCost) const;

            /** \brief Returns whether the edge is valid. */
            bool isValid(const aeitstar::Edge &edge) const;

            /** \brief Returns whether the edge could be valid. */
            bool couldBeValid(const aeitstar::Edge &edge) const;

            /** \brief The sampling-based approximation of the state space. */
            aeitstar::RandomGeometricGraph graph_;

            /** \brief The number of states added when the approximation is updated. */
            std::size_t numSamplesPerBatch_{100u};

            /** \brief The current suboptimality factor of the forward search. */
            double suboptimalityFactor_{std::numeric_limits<float>::infinity()};

            /** \brief The factor that determines when the reverse search should be repaired. */
            double repairFactor_{std::numeric_limits<float>::infinity()};

            /** \brief The option that specifies whether to repair the reverse search when the forward search detects a
             * collision on an edge. */
            bool isRepairingOfReverseTreeEnabled_{false};

            /** \brief The option that specifies whether sparse collision detection on the reverse search tree is enabled. */
            bool isCollisionDetectionInReverseTreeEnabled_{false};

            /** \brief The interpolation values used for the sparse collision detection on the reverse search. */
            std::vector<double> detectionInterpolationValues_{};

            /** \brief The state used to do sparse collision detection with. */
            ompl::base::State *detectionState_;

            /** \brief The cost of the current best solution. */
            ompl::base::Cost bestCost_{std::numeric_limits<double>::signaling_NaN()};

            /** \brief The root of the forward search tree. */
            std::shared_ptr<aeitstar::Vertex> forwardRoot_;

            /** \brief The root of the reverse search tree. */
            std::shared_ptr<aeitstar::Vertex> reverseRoot_;

            /** \brief The forward queue. */
            std::unique_ptr<aeitstar::ForwardQueue> forwardQueue_;

            /** \brief The reverse queue. */
            std::unique_ptr<aeitstar::ReverseQueue> reverseQueue_;

            /** \brief The current iteration. */
            std::size_t iteration_{0u};

            /** \brief The tag of the current search. */
            std::size_t searchTag_{1u};

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
        };

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_AIBITSTAR_AIBITSTAR_

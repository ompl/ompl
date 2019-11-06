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

#include "ompl/geometric/planners/aibitstar/Direction.h"
#include "ompl/geometric/planners/aibitstar/RandomGeometricGraph.h"
#include "ompl/geometric/planners/aibitstar/Queue.h"

namespace ompl
{
    namespace geometric
    {
        class AIBITstar : public ompl::base::Planner
        {
        public:
            /** \brief Constructs an algorithm using the provided space information */
            AIBITstar(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo);

            /** \brief Destructs the algorithm. */
            ~AIBITstar();

            /** \brief Setup the parts of the planner that rely on the problem definition being set. */
            void setup() override;

            /** \brief Solves the provided motion planning problem. */
            ompl::base::PlannerStatus
            solve(const ompl::base::PlannerTerminationCondition &terminationCondition) override;

            /** \brief Gets the best cost of the current solution. */
            ompl::base::Cost bestCost() const;

            /** \brief Sets the number of samples per batch. */
            void setNumSamplesPerBatch(std::size_t numSamples);

            /** \brief Sets the radius factor. */
            void setRadiusFactor(double factor);

            /** \brief Enables collision detection on the reverse search. */
            void enableCollisionDetectionOnReverseSearch(bool enable);

            /** \brief Returns a copy of the forward queue. */
            std::vector<aibitstar::Edge> getForwardQueue() const;

            /** \brief Returns a copy of the reverse queue. */
            std::vector<aibitstar::Edge> getReverseQueue() const;

            /** \brief Returns the edges in the reverse tree. */
            std::vector<aibitstar::Edge> getReverseTree() const;

            /** \brief Returns the next edge in the forward queue. */
            aibitstar::Edge getNextForwardEdge() const;

            /** \brief Returns the next edge in the reverse queue. */
            aibitstar::Edge getNextReverseEdge() const;

            /** \brief Get the planner data. */
            void getPlannerData(base::PlannerData &data) const override;

        private:
            /** \brief Performs one iteration. */
            void iterate();

            /** \brief Perform one reverse iteration. */
            void reverseIterate();

            /** \brief Perform one forward iteration. */
            void forwardIterate();

            /** \brief Updates the solution. */
            void updateSolution() const;

            /** \brief Expands the input state, creating forward edges. */
            std::vector<aibitstar::Edge> forwardExpand(const std::shared_ptr<aibitstar::State> &state) const;

            /** \brief Expands the input state, creating reverse edges. */
            std::vector<aibitstar::Edge> reverseExpand(const std::shared_ptr<aibitstar::State> &state) const;

            /** \brief Creates a forward edge between two states. */
            aibitstar::Edge createForwardEdge(const std::shared_ptr<aibitstar::State> &parent,
                                              const std::shared_ptr<aibitstar::State> &child) const;

            /** \brief Creates a reverse edge between two states. */
            aibitstar::Edge createReverseEdge(const std::shared_ptr<aibitstar::State> &parent,
                                              const std::shared_ptr<aibitstar::State> &child) const;

            /** \brief Computes the forward key for an edge between the two input states. */
            std::array<double, 3u> computeForwardKey(const std::shared_ptr<aibitstar::State> &parent,
                                                     const std::shared_ptr<aibitstar::State> &child,
                                                     const ompl::base::Cost &edgeCost) const;

            /** \brief Computes the reverse key for an edge between the two input states. */
            std::array<double, 3u> computeReverseKey(const std::shared_ptr<aibitstar::State> &parent,
                                                     const std::shared_ptr<aibitstar::State> &child,
                                                     const ompl::base::Cost &edgeCost) const;

            /** \brief Rebuilds the forward queue, recomputing all sort keys. */
            void rebuildForwardQueue();

            /** \brief Returns whether the vertex has been closed during the current search. */
            bool isClosed(const std::shared_ptr<aibitstar::Vertex> &vertex) const;

            /** \brief Returns whether the edge can improve the reverse path. */
            bool canImproveReversePath(const aibitstar::Edge &edge) const;

            /** \brief Returns whether the edge can improve the reverse tree. */
            bool canImproveReverseTree(const aibitstar::Edge &edge) const;

            /** \brief Returns whether the edge can improve the forward path. */
            bool canImproveForwardPath(const aibitstar::Edge &edge) const;

            /** \brief Returns whether the edge can improve the forward tree. */
            bool canImproveForwardTree(const aibitstar::Edge &edge) const;

            /** \brief Returns whether the edge does improve the forward path. */
            bool doesImproveForwardPath(const aibitstar::Edge &edge, const ompl::base::Cost &trueEdgeCost) const;

            /** \brief Returns whether the edge does improve the forward tree. */
            bool doesImproveForwardTree(const aibitstar::Edge &edge, const ompl::base::Cost &trueEdgeCost) const;

            /** \brief Returns whether the edge is valid. */
            bool isValid(const aibitstar::Edge &edge) const;

            /** \brief Returns whether the edge could be valid by performing sparse collision checks. */
            bool couldBeValid(const aibitstar::Edge &edge) const;

            /** \brief The sampling-based approximation of the state space. */
            aibitstar::RandomGeometricGraph graph_;

            /** \brief The number of states added when the approximation is updated. */
            std::size_t numSamplesPerBatch_{100u};

            /** \brief The root of the forward search tree. */
            std::shared_ptr<aibitstar::Vertex> forwardRoot_;

            /** \brief The root of the reverse search tree. */
            std::shared_ptr<aibitstar::Vertex> reverseRoot_;

            /** \brief The forward queue. */
            aibitstar::EdgeQueue<aibitstar::Direction::FORWARD> forwardQueue_;

            /** \brief The reverse queue. */
            aibitstar::EdgeQueue<aibitstar::Direction::REVERSE> reverseQueue_;

            /** \brief The current iteration. */
            std::size_t iteration_{0u};

            /** \brief The tag of the current search. */
            std::size_t searchTag_{1u};

            /** \brief The interpolation values used for the sparse collision detection on the reverse search. */
            std::vector<double> detectionInterpolationValues_{};

            /** \brief Whether the collision detection on the reverse search is enabled. */
            bool isCollisionDetectionOnReverseSearchEnabled_{true};

            /** \brief The state used to do sparse collision detection with. */
            ompl::base::State *detectionState_;

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

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019-present University of Oxford
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

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_AITSTAR_IMPLICITGRAPH_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_AITSTAR_IMPLICITGRAPH_

#include <memory>

#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/Planner.h"

#include "ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h"

#include "ompl/geometric/planners/informedtrees/aitstar/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        namespace aitstar
        {
            class ImplicitGraph
            {
            public:
                /** \brief Constructs an implicit graph. */
                ImplicitGraph(const ompl::base::Cost &solutionCost);

                /** \brief Destructs an implicit graph. */
                virtual ~ImplicitGraph() = default;

                /** \brief The setup method for the graph. Needed to have it on the stack. */
                void setup(const ompl::base::SpaceInformationPtr &spaceInformation,
                           const ompl::base::ProblemDefinitionPtr &problemDefinition,
                           ompl::base::PlannerInputStates *inputStates);

                /** \brief Resets the graph to its construction state, without resetting options. */
                void clear();

                /** \brief Set the rewire factor of the RGG. */
                void setRewireFactor(double rewireFactor);

                /** \brief Get the reqire factor of the RGG. */
                double getRewireFactor() const;

                /** \brief Sets whether to track approximate solutions or not. */
                void setTrackApproximateSolution(bool track);

                /** \brief Adds a batch of samples and returns the samples it has added. */
                std::vector<std::shared_ptr<Vertex>> addSamples(std::size_t numNewSamples);

                /** \brief Gets the number of samples in the graph. */
                std::size_t getNumVertices() const;

                /** \brief Gets the RGG connection radius. */
                double getConnectionRadius() const;

                /** \brief Registers a state as a start state. */
                void registerStartState(const ompl::base::State *const startState);

                /** \brief Registers a state as a goal state. */
                void registerGoalState(const ompl::base::State *const goalState);

                /** \brief Returns whether the graph has a goal state. */
                bool hasAStartState() const;

                /** \brief Returns whether the graph has a goal state. */
                bool hasAGoalState() const;

                /** \brief Adds new start and goals to the graph if avavilable and creates a new informed sampler if
                 * necessary. */
                void updateStartAndGoalStates(const ompl::base::PlannerTerminationCondition &terminationCondition,
                                              ompl::base::PlannerInputStates *inputStates);

                /** \brief Get neighbors of a vertex. */
                std::vector<std::shared_ptr<Vertex>> getNeighbors(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Checks whether the vertex is a start vertex. */
                bool isStart(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Checks whether the vertex is a goal vertex. */
                bool isGoal(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Get the start vertices. */
                const std::vector<std::shared_ptr<Vertex>> &getStartVertices() const;

                /** \brief Get the goal vertices. */
                const std::vector<std::shared_ptr<Vertex>> &getGoalVertices() const;

                /** \brief Get all vertices. */
                std::vector<std::shared_ptr<Vertex>> getVertices() const;

                /** \brief Prune all samples that can not contribute to a solution better than the current one. */
                void prune();

                /** \brief Get the number of state collision checks. */
                std::size_t getNumberOfStateCollisionChecks() const;

                /** \brief Get the number of nearest neighbor calls. */
                std::size_t getNumberOfNearestNeighborCalls() const;

            private:
                /** \brief Computes the number of samples in the informed set. */
                std::size_t computeNumberOfSamplesInInformedSet() const;

                /** \brief Computes the connection radius with a given number of samples. */
                double computeConnectionRadius(std::size_t numSamples) const;

                /** \brief Returns wehther a state can possibly improve the current solution. */
                bool canPossiblyImproveSolution(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief The space information of the underlying planning problem. */
                ompl::base::SpaceInformationPtr spaceInformation_;

                /** \brief The definition of the planning problem. */
                ompl::base::ProblemDefinitionPtr problemDefinition_;

                /** \brief The optimization objective of the planning problem. */
                ompl::base::OptimizationObjectivePtr objective_;

                /** \brief The id of the batch. */
                std::size_t batchId_;

                /** \brief The rewire factor of the RGG. */
                double rewireFactor_{1.0};

                /** \brief Whether to track approximate solutions. */
                bool trackApproximateSolution_{false};

                /** \brief The vertex to the goal. */
                std::shared_ptr<Vertex> bestApproximateGoal_;

                /** \brief The radius that defines the neighborhood of a vertex. */
                double radius_{std::numeric_limits<double>::infinity()};

                /** \brief The cost of the incumbent solution. */
                const ompl::base::Cost& solutionCost_;

                /** \brief The state sampler responsible for filling the state values of vertices. */
                ompl::base::InformedSamplerPtr sampler_;

                /** \brief All vertices in this implicit graph. */
                ompl::NearestNeighborsGNATNoThreadSafety<std::shared_ptr<Vertex>> vertices_;

                /** \brief The start vertices in the graph. */
                std::vector<std::shared_ptr<Vertex>> startVertices_;

                /** \brief The goal vertices in the graph. */
                std::vector<std::shared_ptr<Vertex>> goalVertices_;

                /** \brief The start vertices that have been pruned. They are kept around because if the user decides to
                 * add goal states after we've pruned some start states, we might want to add these pruned start states
                 * again. */
                std::vector<std::shared_ptr<Vertex>> prunedStartVertices_;

                /** \brief The goal vertices that have been pruned. They are kept around because if the user decides to
                 * add start states after we've pruned some goal states, we might want to add these pruned goal states
                 * again. */
                std::vector<std::shared_ptr<Vertex>> prunedGoalVertices_;

                /** \brief The number of state collision checks. */
                mutable std::size_t numStateCollisionChecks_{0u};

                /** \brief The number of state collision checks. */
                mutable std::size_t numNearestNeighborsCalls_{0u};
            };

        }  // namespace aitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_AITSTAR_IMPLICITGRAPH_

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

                /** \brief Set the maximum number of goals AIT* will sample from sampleable goal regions. */
                void setMaxNumberOfGoals(unsigned int maxNumberOfGoals);

                /** \brief Get the maximum number of goals AIT* will sample from sampleable goal regions. */
                unsigned int getMaxNumberOfGoals() const;

                /** \brief Whether to use a k-nearest connection model. If false, it uses an r-disc model. */
                void setUseKNearest(bool useKNearest);

                /** \brief Whether the graph uses a k-nearest connection model. If false, it uses an r-disc model. */
                bool getUseKNearest() const;

                /** \brief Sets whether to track approximate solutions or not. */
                void setTrackApproximateSolution(bool track);

                /** \brief Adds a batch of samples and returns the samples it has added. */
                bool addSamples(std::size_t numNewSamples,
                                const ompl::base::PlannerTerminationCondition &terminationCondition);

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

                /** \brief Returns the total number of sampled states. */
                std::size_t getNumberOfSampledStates() const;

                /** \brief Returns the total number of valid samples found. */
                std::size_t getNumberOfValidSamples() const;

                /** \brief Get the number of state collision checks. */
                std::size_t getNumberOfStateCollisionChecks() const;

                /** \brief Get the number of nearest neighbor calls. */
                std::size_t getNumberOfNearestNeighborCalls() const;

            private:
                /** \brief Computes the number of samples in the informed set. */
                std::size_t computeNumberOfSamplesInInformedSet() const;

                /** \brief Computes the connection radius of the r-disc model with a given number of samples. */
                double computeConnectionRadius(std::size_t numSamples) const;

                /** \brief Computes the number of neighbors of the k-nearest model with a given number of samples. */
                std::size_t computeNumberOfNeighbors(std::size_t numSamples) const;

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

                /** \brief Whether to use a k-nearest RGG. If false, AIT* uses an r-disc RGG. */
                bool useKNearest_{true};

                /** \brief The maximum number of goals BIT* will sample. */
                unsigned int maxNumGoals_{10u};

                /** \brief The radius that defines the neighborhood of a vertex if using an r-disc graph. */
                double radius_{std::numeric_limits<double>::infinity()};

                /** \brief The number of neighbors that defines the neighborhood of a vertex if using a k-nearest graph.
                 */
                std::size_t numNeighbors_{std::numeric_limits<std::size_t>::max()};

                /** \brief A constant for the computation of the number of neighbors when using a k-nearest model. */
                std::size_t k_rgg_{std::numeric_limits<std::size_t>::max()};

                /** \brief The cost of the incumbent solution. */
                const ompl::base::Cost &solutionCost_;

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

                /** \brief The new samples already sampled but not yet added to the nearest neighbor struct. */
                std::vector<std::shared_ptr<Vertex>> newSamples_;

                /** \brief The number of sampled states that were valid. */
                mutable std::size_t numValidSamples_{0u};

                /** \brief The number of sampled states. */
                mutable std::size_t numSampledStates_{0u};

                /** \brief The number of state collision checks. */
                mutable std::size_t numNearestNeighborsCalls_{0u};
            };

        }  // namespace aitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_AITSTAR_IMPLICITGRAPH_

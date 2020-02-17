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

#pragma once

#include <memory>

#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"

#include "ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h"

#include "ompl/geometric/planners/aitstar/datastructures/Vertex.h"

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
                ImplicitGraph();

                /** \brief Deconstructs an implicit graph. */
                virtual ~ImplicitGraph() = default;

                /** \brief The setup method for the graph. Needed to have it on the stack. */
                void setup(const ompl::base::SpaceInformationPtr &spaceInformation,
                           const ompl::base::ProblemDefinitionPtr &problemDefinition,
                           const std::shared_ptr<ompl::base::Cost> &solutionCost,
                           const std::shared_ptr<std::size_t> &forwardSearchId,
                           const std::shared_ptr<std::size_t> &backwardSearchId);

                /** \brief Set the reqire factor of the RGG. */
                void setRewireFactor(double rewireFactor);

                /** \brief Adds a batch of samples. */
                std::vector<std::shared_ptr<Vertex>> addSamples(std::size_t numNewSamples);

                /** \brief Gets the number of samples in the graph. */
                std::size_t getNumVertices() const;

                /** \brief Gets the RGG connection radius. */
                double getConnectionRadius() const;

                /** \brief Registers a state as a start state. */
                void registerStartState(const ompl::base::State *const startState);

                /** \brief Registers a state as a goal state. */
                void registerGoalState(const ompl::base::State *const goalState);

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

            private:
                /** \brief Computes the number of samples in the informed set. */
                std::size_t computeNumberOfSamplesInInformedSet() const;

                /** \brief Computes the connection radius with a given number of samples. */
                double computeConnectionRadius(std::size_t numSamples) const;

                /** \brief The space information of the underlying planning problem. */
                ompl::base::SpaceInformationPtr spaceInformation_;

                /** \brief The definition of the planning problem. */
                ompl::base::ProblemDefinitionPtr problemDefinition_;

                /** \brief The optimization objective of the planning problem. */
                ompl::base::OptimizationObjectivePtr optimizationObjective_;

                /** \brief The id of the batch. */
                std::shared_ptr<std::size_t> batchId_;

                /** \brief The id of the forward search. */
                std::shared_ptr<std::size_t> forwardSearchId_;

                /** \brief The id of the backward search. */
                std::shared_ptr<std::size_t> backwardSearchId_;

                /** \brief The rewire factor of the RGG. */
                double rewireFactor_{1.0};

                /** \brief The radius that defines the neighborhood of a vertex. */
                double radius_{std::numeric_limits<double>::infinity()};

                /** \brief The cost of the incumbent solution. */
                std::weak_ptr<const ompl::base::Cost> solutionCost_;

                /** \brief The state sampler responsible for filling the state values of vertices. */
                ompl::base::InformedSamplerPtr sampler_;

                /** \brief All vertices in this implicit graph. */
                ompl::NearestNeighborsGNATNoThreadSafety<std::shared_ptr<Vertex>> vertices_;

                /** \brief The start vertices in the graph. */
                std::vector<std::shared_ptr<Vertex>> startVertices_;

                /** \brief The goal vertices in the graph. */
                std::vector<std::shared_ptr<Vertex>> goalVertices_;
            };

        }  // namespace aitstar

    }  // namespace geometric

}  // namespace ompl

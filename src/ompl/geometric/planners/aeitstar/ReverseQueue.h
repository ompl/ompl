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

#ifndef OMPL_GEOMETRIC_PLANNERS_AIBITSTAR_REVERSE_QUEUE_
#define OMPL_GEOMETRIC_PLANNERS_AIBITSTAR_REVERSE_QUEUE_

#include <array>
#include <map>

#include "ompl/base/Cost.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/datastructures/BinaryHeap.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include "ompl/geometric/planners/aeitstar/Direction.h"
#include "ompl/geometric/planners/aeitstar/Edge.h"
#include "ompl/geometric/planners/aeitstar/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        namespace aeitstar
        {
            class ReverseQueue
            {
            public:
                /** \brief Constructs the queue. */
                ReverseQueue(const std::shared_ptr<const ompl::base::OptimizationObjective> &objective);

                /** \brief Destructs the queue. */
                ~ReverseQueue() = default;

                /** \brief Returns whether the queue is empty. */
                bool empty() const;

                /** \brief Returns how many elements are in the queue. */
                std::size_t size() const;

                /** \brief Insert an element into the queue. */
                void insert(const Edge &edge);

                /** \brief Insert an element into the queue. */
                void insert(const std::vector<Edge> &edges);

                /** Get a reference to the top edge in the queue. */
                const Edge &peek() const;

                /** \brief Returns and deletes the top element of the queue. */
                Edge pop();

                /** \brief Clears the queue, i.e., deletes all elements from it. */
                void clear();

                /** \brief Copies all edges into a vector and returns the vector. */
                std::vector<Edge> getEdges() const;

                /** \brief Rebuilds the queue. */
                void rebuild();

                void removeOutgoingEdges(const std::shared_ptr<Vertex> &vertex);

            private:
                /** \brief Update an edge in the queue if it exists. */
                bool update(const Edge &edge);

                /** \brief The optimization objective. */
                std::shared_ptr<const ompl::base::OptimizationObjective> objective_;

                /** \brief The state space information. */
                std::shared_ptr<const ompl::base::SpaceInformation> spaceInfo_;

                /** \brief The queue is ordered on the lower bound cost through an edge. */
                using CostHeap =
                    ompl::BinaryHeap<std::pair<std::array<ompl::base::Cost, 2u>, Edge>,
                                     std::function<bool(const std::pair<std::array<ompl::base::Cost, 2u>, Edge> &,
                                                        const std::pair<std::array<ompl::base::Cost, 2u>, Edge> &)>>;
                CostHeap queue_;
            };
        }  // namespace aeitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_AIBITSTAR_REVERSE_QUEUE_

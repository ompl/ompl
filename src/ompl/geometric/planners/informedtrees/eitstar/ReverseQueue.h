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

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EITSTAR_REVERSE_QUEUE_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EITSTAR_REVERSE_QUEUE_

#include <array>
#include <map>
#include <tuple>

#include "ompl/base/Cost.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/datastructures/BinaryHeap.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include "ompl/geometric/planners/informedtrees/eitstar/Direction.h"
#include "ompl/geometric/planners/informedtrees/eitstar/Edge.h"
#include "ompl/geometric/planners/informedtrees/eitstar/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        namespace eitstar
        {
            class ReverseQueue
            {
            public:
                /** \brief Constructs the queue with the given optimization objective and state space. */
                ReverseQueue(const std::shared_ptr<const ompl::base::OptimizationObjective> &objective,
                             const std::shared_ptr<const ompl::base::StateSpace> &space, const bool isQueueCostOrdered);

                /** \brief Destructs this queue. */
                ~ReverseQueue() = default;

                /** \brief Returns whether the queue is empty. */
                bool empty() const;

                /** \brief Returns the number of elements in the queue. */
                std::size_t size() const;

                /** \brief Inserts or updates an element in the queue. */
                void insertOrUpdate(const Edge &edge);

                /** \brief Inserts or updates multiple elements in the queue. */
                void insertOrUpdate(const std::vector<Edge> &edges);

                /** \brief Get a reference to the top edge in the queue. */
                const Edge &peek() const;

                /** \brief Get the effort corresponding to the top edge of the queue. */
                unsigned int peekEffort() const;

                /** \brief Returns and deletes the top element of the queue. */
                Edge pop();

                /** \brief Updates the queue ordering depending on the given suboptimality factor. */
                void setCostQueueOrder(const bool isQueueCostOrdered);

                /** \brief Returns a lower bound on the resolution-optimal solution cost. */
                ompl::base::Cost getLowerBoundOnOptimalSolutionCost() const;

                /** \brief Clears the queue, i.e., deletes all elements from it. */
                void clear();

                /** \brief Copies all edges into a vector and returns the vector. */
                std::vector<Edge> getEdges() const;

                /** \brief Rebuilds the queue. */
                void rebuild();

                /** \brief Removes the outgoing edges of a vertex from the queue. */
                void removeOutgoingEdges(const std::shared_ptr<Vertex> &vertex);

                /** \brief Returns the admissible total potential effort of an edge. */
                unsigned int computeAdmissibleSolutionEffort(const Edge &edge) const;

                /** \brief Returns the admissible total potential solution cost of an edge. */
                ompl::base::Cost computeAdmissibleSolutionCost(const Edge &edge) const;

            private:
                using HeapElement = std::tuple<ompl::base::Cost, ompl::base::Cost, unsigned int, unsigned int, Edge>;
                using CostEffortHeap =
                    ompl::BinaryHeap<HeapElement, std::function<bool(const HeapElement &, const HeapElement &)>>;

                /** \brief Update an edge in the queue if it exists. */
                bool updateIfExists(const Edge &edge);

                /** \brief Returns the cost to come to the target of the edge. */
                ompl::base::Cost computeAdmissibleCostToComeToTarget(const Edge &edge) const;

                /** \brief Returns the inadmissible total potential solution effort of an edge. */
                unsigned int computeInadmissibleSolutionEffort(const Edge &edge) const;

                /** \brief Returns the comparison operator that orders on cost with an effort tiebreaker. */
                std::function<bool(const HeapElement &, const HeapElement &)> getCostComparisonOperator() const;

                /** \brief Returns the comparison operator that orders on effort with a cost tiebreaker. */
                std::function<bool(const HeapElement &, const HeapElement &)> getEffortComparisonOperator() const;

                /** \brief What the reverse queue is ordered on. */
                bool isQueueCostOrdered_;

                /** \brief The optimization objective. */
                std::shared_ptr<const ompl::base::OptimizationObjective> objective_;

                /** \brief The state space information. */
                std::shared_ptr<const ompl::base::StateSpace> space_;

                /** \brief The queue is ordered on [g + c + h, g + c, effort] */
                CostEffortHeap queue_;
            };
        }  // namespace eitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EITSTAR_REVERSE_QUEUE_

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

#ifndef OMPL_GEOMETRIC_PLANNERS_EITSTAR_FORWARD_QUEUE_
#define OMPL_GEOMETRIC_PLANNERS_EITSTAR_FORWARD_QUEUE_

#include <array>
#include <map>
#include <utility>
#include <vector>

#include "ompl/base/Cost.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/datastructures/BinaryHeap.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include "ompl/geometric/planners/eitstar/Direction.h"
#include "ompl/geometric/planners/eitstar/Edge.h"
#include "ompl/geometric/planners/eitstar/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        namespace eitstar
        {
            class ForwardQueue
            {
            public:
                /** \brief Constructs the queue. */
                ForwardQueue(const std::shared_ptr<const ompl::base::OptimizationObjective> &objective,
                             const std::shared_ptr<const ompl::base::SpaceInformation> &spaceInfo);

                /** \brief Destructs the queue. */
                ~ForwardQueue() = default;

                /** \brief Returns whether the queue is empty. */
                bool empty() const;

                /** \brief Returns how many elements are in the queue. */
                std::size_t size() const;

                /** \brief Insert an edge into the queue. */
                void insert(const Edge &edge);

                /** \brief Inserts multiple edges into the queue. */
                void insert(const std::vector<Edge> &edges);

                /** \brief Removes an edge from the queue. Throws if the edge is not in the queue. */
                void remove(const Edge &edge);

                /** \brief Update an edge in the queue. */
                bool update(const Edge &edge);

                /** \brief Returns a copy to the next edge. */
                Edge peek(double suboptimalityFactor) const;

                /** \brief Returns and deletes the top element of the queue. */
                Edge pop(double suboptimalityFactor);

                /** \brief Returns a lower bound on the resolution-optimal solution cost. */
                ompl::base::Cost getLowerBoundOnOptimalSolutionCost() const;

                /** \brief Clears the queue, i.e., deletes all elements from it. */
                void clear();

                /** \brief Copies all edges into a vector and returns the vector. */
                std::vector<Edge> getEdges() const;

                /** \brief Rebuilds the queue. */
                void rebuild();

            private:
                /** \brief Estimates the effort that remains to validate a solution through an edge. */
                std::size_t estimateEffort(const Edge &edge) const;

                /** \brief Estimates the cost of a solution through an edge (possibly inadmissible). */
                ompl::base::Cost estimateCost(const Edge &edge) const;

                /** \brief Returns a lower bounding cost for a solution through an edge (admissible). */
                ompl::base::Cost lowerBoundCost(const Edge &edge) const;

                /** \brief The optimization objective. */
                std::shared_ptr<const ompl::base::OptimizationObjective> objective_;

                /** \brief The state space information. */
                std::shared_ptr<const ompl::base::SpaceInformation> spaceInfo_;

                /** \brief The three values an edge can be sorted by. */
                struct EdgeKeys
                {
                    EdgeKeys(const ompl::base::Cost& lowerBound, const ompl::base::Cost& estimated, const std::size_t effort)
                      : lowerBoundCost(lowerBound), estimatedCost(estimated), estimatedEffort(effort){};
                    ompl::base::Cost lowerBoundCost;
                    ompl::base::Cost estimatedCost;
                    std::size_t estimatedEffort;
                };

                /** \brief The queue is ordered on the lower bound cost through an edge. */
                std::vector<std::pair<EdgeKeys, Edge>> queue_;
            };
        }  // namespace eitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_EITSTAR_FORWARD_QUEUE_

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

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EITSTAR_FORWARD_QUEUE_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EITSTAR_FORWARD_QUEUE_

#include <array>
#include <map>
#include <utility>
#include <vector>

#include "ompl/base/Cost.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/datastructures/BinaryHeap.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include "ompl/geometric/planners/informedtrees/eitstar/Direction.h"
#include "ompl/geometric/planners/informedtrees/eitstar/Edge.h"
#include "ompl/geometric/planners/informedtrees/eitstar/Vertex.h"

#include <unordered_map>

namespace ompl
{
    namespace geometric
    {
        namespace eitstar
        {
            struct pair_hash
            {
                std::size_t operator()(const std::pair<std::size_t, std::size_t> &k) const
                {
                    // This method to combine hashes is the same as boost::hash_combine
                    // https://www.boost.org/doc/libs/1_67_0/boost/container_hash/hash.hpp
                    // For a discussion see e.g. https://stackoverflow.com/a/35991300
                    std::hash<std::size_t> hasher;

                    std::size_t seed = hasher(k.first);
                    seed ^= hasher(k.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

                    return seed;
                }
            };

            class ForwardQueue
            {
            public:
                /** \brief Constructs the queue given the objective and state space. */
                ForwardQueue(const std::shared_ptr<const ompl::base::OptimizationObjective> &objective,
                             const std::shared_ptr<const ompl::base::StateSpace> &space);

                /** \brief Destructs the queue. */
                ~ForwardQueue() = default;

                /** \brief Returns whether the queue is empty. */
                bool empty() const;

                /** \brief Returns how many elements are in the queue. */
                std::size_t size() const;

                /** \brief Inserts or updates an edge in the queue. */
                void insertOrUpdate(const Edge &edge);

                /** \brief Inserts or updates multiple edges into the queue. */
                void insertOrUpdate(const std::vector<Edge> &edges);

                /** \brief Removes an edge from the queue. Throws if the edge is not in the queue. */
                void remove(const Edge &edge);

                /** \brief Update an edge in the queue. Does nothing if the edge is not in the queue. */
                void updateIfExists(const Edge &edge);

                /** \brief Returns a copy to the next edge. */
                Edge peek(double suboptimalityFactor);

                /** \brief Returns and deletes the top element of the queue. */
                Edge pop(double suboptimalityFactor);

                /** \brief Returns a lower bound on the resolution-optimal solution cost. */
                ompl::base::Cost getLowerBoundOnOptimalSolutionCost() const;

                /** \brief Returns whether the queue contains edges with targets that are open or unconnected in the
                 * reverse search tree. */
                bool containsOpenTargets(std::size_t reverseSearchTag) const;

                /** \brief Clears the queue, i.e., deletes all elements from it. */
                void clear();

                /** \brief Copies all edges into a vector and returns the vector. */
                std::vector<Edge> getEdges() const;

                /** \brief Rebuilds the queue. */
                void rebuild();

                /** \brief Returns the minimum effort that remains. */
                unsigned int getMinEffortToCome() const;

                /** \brief Estimates the effort that remains to validate a solution through an edge. */
                std::size_t estimateEffort(const Edge &edge) const;

            private:
                /** \brief The three values an edge can be sorted by. */
                struct EdgeKeys
                {
                    EdgeKeys(const ompl::base::Cost &lowerBound, const ompl::base::Cost &estimated,
                             const std::size_t effort)
                      : lowerBoundCost(lowerBound), estimatedCost(estimated), estimatedEffort(effort){};
                    ompl::base::Cost lowerBoundCost;
                    ompl::base::Cost estimatedCost;
                    std::size_t estimatedEffort;
                };

                /** \brief Creates a queue element from the given edge. */
                std::pair<EdgeKeys, Edge> makeElement(const Edge &edge) const;

                using Container =
                    std::unordered_map<std::pair<std::size_t, std::size_t>, std::pair<EdgeKeys, Edge>, pair_hash>;

                /** \brief Finds the iterator at the front of the queue. */
                Container::const_iterator getFrontIter(double suboptimalityFactor);

                /** \brief Returns the edge pair from the container. */
                inline std::pair<EdgeKeys, Edge> &get(Container::iterator &it) const
                {
                    return it->second;
                }

                /** \brief Returns the edge pair from the container. */
                inline const std::pair<EdgeKeys, Edge> &get(Container::const_iterator &it) const
                {
                    return it->second;
                }

                /** \brief Returns an iterator to the edge with the best estimated cost. */
                Container::iterator getBestCostEstimateEdge();

                /** \brief Returns an iterator to the edge with the lower bound cost. */
                Container::iterator getLowerBoundCostEdge();

                /** \brief Returns a constant iterator to the edge with the lower bound cost. */
                Container::const_iterator getLowerBoundCostEdge() const;

                /** \brief Returns a constant iterator to the edge with the best estimated cost. */
                Container::const_iterator getBestCostEstimateEdge() const;

                /** \brief Returns the cost inflated by a factor. */
                ompl::base::Cost inflateCost(const ompl::base::Cost &cost, double factor) const;

                /** \brief Returns the estimated cost of a solution through an edge (possibly inadmissible). */
                ompl::base::Cost estimateCost(const Edge &edge) const;

                /** \brief Returns a lower bounding cost for a solution through an edge (admissible). */
                ompl::base::Cost lowerBoundCost(const Edge &edge) const;

                /** \brief The optimization objective. */
                std::shared_ptr<const ompl::base::OptimizationObjective> objective_;

                /** \brief The state space. */
                std::shared_ptr<const ompl::base::StateSpace> space_;

                /** \brief The queue does not maintain order, the peek/pop methods give the top element of the queue. */
                Container queue_{};

                /** \brief Iterator to the current top element in the queue. */
                Container::const_iterator front_;

                /** \brief Indicates whether the lookup of the top elelement should be cached. */
                const bool cacheQueueLookup_ = true;

                /** \brief Indicates if the queue was changed since the last time the top element was cached. */
                bool modifiedQueue_ = true;

                /** \brief The cached minimum effort. */
                mutable unsigned int cachedMinEdgeEffort_{0u};
            };
        }  // namespace eitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EITSTAR_FORWARD_QUEUE_

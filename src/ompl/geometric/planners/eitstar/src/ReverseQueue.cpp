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

#include "ompl/geometric/planners/eitstar/ReverseQueue.h"

#include <utility>

#include "ompl/geometric/planners/eitstar/Direction.h"
#include "ompl/geometric/planners/eitstar/State.h"

namespace ompl
{
    namespace geometric
    {
        namespace eitstar
        {
            ReverseQueue::ReverseQueue(const std::shared_ptr<const ompl::base::OptimizationObjective> &objective,
                                       const std::shared_ptr<const ompl::base::StateSpace> &space)
              : objective_(objective)
              , space_(space)
              , queue_([objective](const HeapElement &lhs, const HeapElement &rhs) {
                  if (objective->isCostEquivalentTo(std::get<0>(lhs), std::get<0>(rhs)))
                  {
                      return std::get<1>(lhs) < std::get<1>(rhs);
                  }
                  else
                  {
                      return objective->isCostBetterThan(std::get<0>(lhs), std::get<0>(rhs));
                  }
              })
            {
            }

            bool ReverseQueue::empty() const
            {
                return size() == 0u;
            }

            std::size_t ReverseQueue::size() const
            {
                return queue_.size();
            }

            void ReverseQueue::insertOrUpdate(const Edge &edge)
            {
                if (!updateIfExists(edge))
                {
                    // Compute the keys.
                    const auto key1 = computeAdmissibleSolutionCost(edge);
                    const auto key2 = computeAdmissibleSolutionEffort(edge);

                    // Create the heap element.
                    const auto element = std::make_tuple(key1, key2, edge);

                    // Insert the edge with the key in the queue.
                    const auto elementPointer = queue_.insert(element);

                    // Remember the element.
                    edge.source->asReverseVertex()->outgoingReverseQueueLookup_.emplace_back(elementPointer);
                }
            }

            void ReverseQueue::insertOrUpdate(const std::vector<Edge> &edges)
            {
                // Let's do this naively.
                for (const auto &edge : edges)
                {
                    insertOrUpdate(edge);
                }
            }

            const Edge &ReverseQueue::peek() const
            {
                if (auto element = queue_.top())
                {
                    return std::get<2>(element->data);
                }
                else
                {
                    throw std::out_of_range("There are no elements to peek in the reverse queue.");
                }
            }

            bool ReverseQueue::updateIfExists(const Edge &edge)
            {
                // Check if the edge is in the queue via the reverse queue pointers.
                const auto &lookup = edge.source->asReverseVertex()->outgoingReverseQueueLookup_;
                const auto it = std::find_if(lookup.cbegin(), lookup.cend(), [&edge](const auto &p) {
                    return std::get<2>(p->data).target->getId() == edge.target->getId();
                });

                // Indicate that the edge is not in the queue by returning false.
                if (it == lookup.cend())
                {
                    return false;
                }

                // Update the cost and effort and the position of the edge in the queue.
                std::get<0>((*it)->data) = computeAdmissibleSolutionCost(edge);
                std::get<1>((*it)->data) = computeAdmissibleSolutionEffort(edge);
                queue_.update(*it);

                // Indicate that the edge was updated by returning true.
                return true;
            }

            ompl::base::Cost ReverseQueue::computeAdmissibleSolutionCost(const Edge &edge) const
            {
                return objective_->combineCosts(
                    objective_->combineCosts(edge.source->getAdmissibleCostToGo(),
                                             objective_->motionCostHeuristic(edge.target->raw(), edge.source->raw())),
                    edge.target->getLowerBoundCostToCome());
            }

            unsigned int ReverseQueue::computeAdmissibleSolutionEffort(const Edge &edge) const
            {
                return edge.source->getEstimatedEffortToGo() +
                       space_->validSegmentCount(edge.target->raw(), edge.source->raw()) +
                       edge.target->getLowerBoundEffortToCome();
            }

            Edge ReverseQueue::pop()
            {
                assert(!queue_.empty());

                // Get the top element, i.e., a pair that holds the key and the edge.
                const auto element = queue_.top();

                // Copy the data of the top edge.
                auto edge = std::get<2>(element->data);

                // If the source state of the edge does not have an associated vertex, it's a bug.
                assert(edge.source->hasReverseVertex());

                // Pop the element from the queue.
                queue_.pop();

                // Get a reference to the outgoing edge queue lookup of the parent vertex.
                auto &lookup = edge.source->asReverseVertex()->outgoingReverseQueueLookup_;

                // Remove the edge from the lookup.
                auto it = std::find(lookup.begin(), lookup.end(), element);

                // If the edge is not in the lookup, it's a bug.
                assert(it != lookup.end());

                // Swappedy pop.
                std::iter_swap(it, lookup.rbegin());
                lookup.pop_back();

                // Return the element.
                return edge;
            }

            ompl::base::Cost ReverseQueue::getLowerBoundOnOptimalSolutionCost() const
            {
                return std::get<0>(queue_.top()->data);
            }

            void ReverseQueue::clear()
            {
                // Can't use queue_.clear() because we need to remove the outgoing edge lookup pointers.
                while (!empty())
                {
                    pop();
                }
            }

            std::vector<Edge> ReverseQueue::getEdges() const
            {
                std::vector<HeapElement> contents;
                queue_.getContent(contents);
                std::vector<Edge> edges;
                edges.reserve(contents.size());
                for (const auto &element : contents)
                {
                    edges.push_back(std::get<2>(element));
                }
                return edges;
            }

            void ReverseQueue::rebuild()
            {
                const auto edges = getEdges();
                clear();
                insertOrUpdate(edges);
            }

            void ReverseQueue::removeOutgoingEdges(const std::shared_ptr<Vertex> &vertex)
            {
                // Remove all elements from the queue.
                for (const auto element : vertex->outgoingReverseQueueLookup_)
                {
                    queue_.remove(element);
                }

                // Remove all elements from the lookup.
                vertex->outgoingReverseQueueLookup_.clear();
            }

        }  // namespace eitstar

    }  // namespace geometric

}  // namespace ompl
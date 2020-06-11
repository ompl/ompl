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
            ReverseQueue::ReverseQueue(const std::shared_ptr<const ompl::base::OptimizationObjective> &objective)
              : objective_(objective)
              , queue_([objective](const std::pair<std::array<ompl::base::Cost, 2u>, Edge> &lhs,
                                   const std::pair<std::array<ompl::base::Cost, 2u>, Edge> &rhs) {
                  return std::lexicographical_compare(
                      lhs.first.begin(), lhs.first.end(), rhs.first.begin(), rhs.first.end(),
                      [objective](const ompl::base::Cost &lhs, const ompl::base::Cost &rhs) {
                          return objective->isCostBetterThan(lhs, rhs);
                      });
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

            void ReverseQueue::insert(const Edge &edge)
            {
                if (!update(edge))
                {
                    // Compute the first field of the key.
                    auto key1 =
                        objective_->combineCosts(objective_->combineCosts(edge.source->getAdmissibleCostToGo(),
                                                                          objective_->motionCostHeuristic(
                                                                              edge.source->raw(), edge.target->raw())),
                                                 edge.target->getLowerBoundCostToCome());

                    // Compute the second field of the key.
                    auto key2 = objective_->combineCosts(
                        edge.source->getAdmissibleCostToGo(),
                        objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw()));

                    // Combine the two fields into the key.
                    std::array<ompl::base::Cost, 2u> key = {key1, key2};

                    // Create the key, edge pair.
                    auto keyEdgePair = std::make_pair(key, edge);

                    // Insert the edge with the key in the queue.
                    auto element = queue_.insert(keyEdgePair);

                    // Remember the element.
                    edge.source->asReverseVertex()->outgoingReverseQueueLookup_.emplace_back(element);
                }
            }

            void ReverseQueue::insert(const std::vector<Edge> &edges)
            {
                // Let's do this naively.
                for (const auto &edge : edges)
                {
                    insert(edge);
                }
            }

            const Edge &ReverseQueue::peek() const
            {
                if (auto element = queue_.top())
                {
                    return element->data.second;
                }
                else
                {
                    throw std::out_of_range("There are no elements to peek in the reverse queue.");
                }
            }

            bool ReverseQueue::update(const Edge &edge)
            {
                // Update if the edges is already in the queue.
                for (const auto outgoingEdge : edge.source->asReverseVertex()->outgoingReverseQueueLookup_)
                {
                    if (outgoingEdge->data.second.target->getId() == edge.target->getId())
                    {
                        auto oldCost = outgoingEdge->data.first;
                        std::array<ompl::base::Cost, 2u> newCost{
                            objective_->combineCosts(
                                objective_->combineCosts(
                                    edge.source->getAdmissibleCostToGo(),
                                    objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw())),
                                edge.target->getLowerBoundCostToGo()),
                            objective_->combineCosts(
                                edge.source->getAdmissibleCostToGo(),
                                objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw()))};
                        if (std::lexicographical_compare(
                                oldCost.begin(), oldCost.end(), newCost.begin(), newCost.end(),
                                [this](const ompl::base::Cost &lhs, const ompl::base::Cost &rhs) {
                                    return objective_->isCostBetterThan(lhs, rhs);
                                }))
                        {
                            outgoingEdge->data.first = newCost;
                            queue_.update(outgoingEdge);
                        }
                        return true;
                    }
                }
                return false;
            }

            Edge ReverseQueue::pop()
            {
                assert(!queue_.empty());

                // Get the top element, i.e., a pair that holds the key and the edge.
                auto element = queue_.top();

                // Copy the data of the top edge.
                auto edge = element->data.second;

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
                std::vector<std::pair<std::array<ompl::base::Cost, 2u>, Edge>> contents;
                queue_.getContent(contents);
                std::vector<Edge> edges;
                edges.reserve(contents.size());
                for (const auto &element : contents)
                {
                    edges.push_back(element.second);
                }
                return edges;
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

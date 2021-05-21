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

#include "ompl/geometric/planners/eitstar/ForwardQueue.h"

#include <algorithm>

#include "ompl/geometric/planners/eitstar/stopwatch/timetable.h"
#include "ompl/geometric/planners/eitstar/Direction.h"
#include "ompl/geometric/planners/eitstar/State.h"

namespace ompl
{
    namespace geometric
    {
        namespace eitstar
        {
            ForwardQueue::ForwardQueue(const std::shared_ptr<const ompl::base::OptimizationObjective> &objective,
                                       const std::shared_ptr<const ompl::base::StateSpace> &space)
              : objective_(objective), space_(space), queue_()
            {
                queue_.reserve(1000u);
            }

            bool ForwardQueue::empty() const
            {
                return size() == 0u;
            }

            std::size_t ForwardQueue::size() const
            {
                return queue_.size();
            }

            void ForwardQueue::insertOrUpdate(const Edge &edge)
            {
                // We update by removing the old edge and inserting the new.
                auto iter = iterator(edge);
                if (iter != queue_.end())
                {
                    queue_.erase(iter);
                }

                // Create the queue element of the edge.
                const auto element = makeElement(edge);

                // Find where the edge belongs.
                const auto pos = position(element);

                // Insert it at the correct position.
                queue_.insert(pos, element);

                // Add it to the cache in the target state.
                edge.target->addToSourcesOfIncomingEdgesInForwardQueue(edge.source);
            }

            void ForwardQueue::insertOrUpdate(const std::vector<Edge> &edges)
            {
                // For now, let's just do this naively.
                for (const auto &edge : edges)
                {
                    insertOrUpdate(edge);
                }
            }

            void ForwardQueue::remove(const Edge &edge)
            {
                const auto it = iterator(edge);

                if (it != queue_.cend())
                {
                    // The forward queue does not have a lookup, just erase the edge.
                    queue_.erase(it);
                    edge.target->removeFromSourcesOfIncomingEdgesInForwardQueue(edge.source);
                }
                else
                {
                    throw std::out_of_range("Can not remove edge from the forward queue, because it is not in the "
                                            "queue.");
                }
            }

            Edge ForwardQueue::peek(double suboptimalityFactor)
            {
                // Make sure the queue contains edges.
                if (queue_.empty())
                {
                    throw std::out_of_range("Forward queue is empty, cannot peek.");
                }
                
                const auto edge = pop(suboptimalityFactor);
                insertOrUpdate(edge);
                return edge;
            } 

            void ForwardQueue::updateIfExists(const Edge &edge)
            {
                // Get an iterator to the element in the queue.
                const auto iter = iterator(edge);

                // If the element doesn't exist in the queue, we return.
                if (iter == queue_.end())
                {
                    return;
                }

                // We update by erasing and inserting at the correct location.
                queue_.erase(iter);

                // Create the key and edge pair.
                const auto element = makeElement(edge);

                // Find where to insert the pair.
                const auto pos = position(element);

                // Insert the edge at the correct location.
                queue_.insert(pos, element);
            }

            Edge ForwardQueue::pop(double suboptimalityFactor)
            {
                // Get the lower bounding edge and corresponding cost.
                const auto lowerBoundEdge = queue_.begin();
                const auto lowerBoundEdgeCost = inflateCost(lowerBoundEdge->first.lowerBoundCost, suboptimalityFactor);

                // Find the best estimate edge and corresponding cost.
                const auto bestCostEdge = getBestCostEstimateEdge();
                const auto bestCostEdgeCost = inflateCost(bestCostEdge->first.estimatedCost, suboptimalityFactor);

                // Find the least effort edge and corresponding cost.
                auto bestEffortEdge = queue_.cbegin();
                auto bestEffortEdgeCost = objective_->infiniteCost();
                for (auto it = queue_.cbegin(); it != queue_.cend(); ++it)
                {
                    if (it->first.estimatedEffort < bestEffortEdge->first.estimatedEffort &&
                        !objective_->isCostBetterThan(bestCostEdgeCost, it->first.estimatedCost))
                    {
                        bestEffortEdgeCost = it->first.estimatedCost;
                        bestEffortEdge = it;
                    }
                }

                // Return the correct edge.
                if (!objective_->isCostBetterThan(lowerBoundEdgeCost, bestEffortEdgeCost))
                {
                    const auto edge = bestEffortEdge->second;
                    queue_.erase(bestEffortEdge);
                    edge.target->removeFromSourcesOfIncomingEdgesInForwardQueue(edge.source);
                    return edge;
                }
                else if (!objective_->isCostBetterThan(lowerBoundEdgeCost, bestCostEdge->first.estimatedCost))
                {
                    const auto edge = bestCostEdge->second;
                    edge.target->removeFromSourcesOfIncomingEdgesInForwardQueue(edge.source);
                    queue_.erase(bestCostEdge);
                    return edge;
                }
                else
                {
                    const auto edge = lowerBoundEdge->second;
                    edge.target->removeFromSourcesOfIncomingEdgesInForwardQueue(edge.source);
                    queue_.erase(lowerBoundEdge);
                    return edge;
                }
            }

            ompl::base::Cost ForwardQueue::getLowerBoundOnOptimalSolutionCost() const
            {
                return queue_.begin()->first.lowerBoundCost;
            }

            void ForwardQueue::clear()
            {
                for (const auto &edge : queue_)
                {
                    edge.second.target->resetSourcesOfIncomingEdgesInForwardQueue();
                }
                queue_.clear();
            }

            std::vector<Edge> ForwardQueue::getEdges() const
            {
                std::vector<Edge> edges;
                edges.reserve(queue_.size());
                for (auto keyAndEdge : queue_)
                {
                    edges.emplace_back(keyAndEdge.second);
                }
                return edges;
            }

            void ForwardQueue::rebuild()
            {
                const auto edges = getEdges();
                clear();
                insertOrUpdate(edges);
            }

            std::pair<ForwardQueue::EdgeKeys, Edge> ForwardQueue::makeElement(const Edge &edge) const
            {
                // Compute the costs and effort.
                const auto lowerBoundCostOfEdge = lowerBoundCost(edge);
                const auto estimatedCostOfEdge = estimateCost(edge);
                const auto estimatedEffortOfEdge = estimateEffort(edge);

                return {{lowerBoundCostOfEdge, estimatedCostOfEdge, estimatedEffortOfEdge}, {edge.source, edge.target}};
            }

            std::vector<std::pair<ForwardQueue::EdgeKeys, Edge>>::iterator ForwardQueue::iterator(const Edge &edge)
            {
                // Should probably make use of the fact that the queue is sorted. Look into std::binary_search.
                return std::find_if(queue_.begin(), queue_.end(), [&edge](const auto &element) {
                    return edge.source->getId() == element.second.source->getId() &&
                           edge.target->getId() == element.second.target->getId();
                });
            }

            std::vector<std::pair<ForwardQueue::EdgeKeys, Edge>>::iterator
            ForwardQueue::position(const std::pair<EdgeKeys, Edge> &keysAndEdge)
            {
                return std::lower_bound(
                    queue_.begin(), queue_.end(), keysAndEdge, [this](const auto &a, const auto &b) {
                        return objective_->isCostBetterThan(a.first.lowerBoundCost, b.first.lowerBoundCost);
                    });
            }

            std::vector<std::pair<ForwardQueue::EdgeKeys, Edge>>::iterator ForwardQueue::getBestCostEstimateEdge()
            {
                // Find the best estimate edge and corresponding cost.
                return std::min_element(queue_.begin(), queue_.end(), [this](const auto &a, const auto &b) {
                    return objective_->isCostBetterThan(a.first.estimatedCost, b.first.estimatedCost);
                });
            }

            std::vector<std::pair<ForwardQueue::EdgeKeys, Edge>>::const_iterator
            ForwardQueue::getBestCostEstimateEdge() const
            {
                // Find the best estimate edge and corresponding cost.
                return std::min_element(queue_.cbegin(), queue_.cend(), [this](const auto &a, const auto &b) {
                    return objective_->isCostBetterThan(a.first.estimatedCost, b.first.estimatedCost);
                });
            }

            ompl::base::Cost ForwardQueue::inflateCost(const ompl::base::Cost &cost, double factor) const
            {
                // I don't think this will work for problems which minimize cost, but making it work would require a
                // larger change in OMPL.
                return ompl::base::Cost(cost.value() * factor);
            }

            std::size_t ForwardQueue::estimateEffort(const Edge &edge) const
            {
                return space_->validSegmentCount(edge.source->raw(), edge.target->raw()) +
                       edge.target->getEstimatedEffortToGo();
            }

            ompl::base::Cost ForwardQueue::estimateCost(const Edge &edge) const
            {
                return objective_->combineCosts(objective_->combineCosts(edge.source->getCurrentCostToCome(),
                                                                         objective_->motionCostBestEstimate(
                                                                             edge.source->raw(), edge.target->raw())),
                                                edge.target->getEstimatedCostToGo());
            }

            ompl::base::Cost ForwardQueue::lowerBoundCost(const Edge &edge) const
            {
                return objective_->combineCosts(
                    objective_->combineCosts(edge.source->getCurrentCostToCome(),
                                             objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw())),
                    edge.target->getAdmissibleCostToGo());
            }

        }  // namespace eitstar

    }  // namespace geometric

}  // namespace ompl

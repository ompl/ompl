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
                                       const std::shared_ptr<const ompl::base::SpaceInformation> &spaceInfo)
              : objective_(objective), spaceInfo_(spaceInfo), queue_()
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

            void ForwardQueue::insert(const Edge &edge)
            {
                // Compute the costs and effort.
                const auto lowerBoundCostOfEdge = lowerBoundCost(edge);
                const auto estimatedCostOfEdge = estimateCost(edge);
                const auto estimatedEffortOfEdge = estimateEffort(edge);

                const auto keysAndEdge = std::make_pair<EdgeKeys, Edge>(
                    {lowerBoundCostOfEdge, estimatedCostOfEdge, estimatedEffortOfEdge}, {edge.source, edge.target});

                // Find where to insert the edge.
                const auto it = std::lower_bound(
                    queue_.cbegin(), queue_.cend(), keysAndEdge,
                    [this](const std::pair<EdgeKeys, Edge> &a, const std::pair<EdgeKeys, Edge> &b) {
                        return objective_->isCostBetterThan(a.first.lowerBoundCost, b.first.lowerBoundCost);
                    });

                // Check if the element already exists.
                auto element =
                    std::find_if(queue_.cbegin(), queue_.cend(), [&edge](const std::pair<EdgeKeys, Edge> &element) {
                        return edge.source->getId() == element.second.source->getId() &&
                               edge.target->getId() == element.second.target->getId();
                    });
                if (element != queue_.end())
                {
                    // The element exists.
                    if (element == it)
                    {
                        // The element exists and is already at the right position, we're done.
                        return;
                    }
                    else  // If the element exists but is not at the right place, erase it.
                    {
                        queue_.erase(element);
                    }
                }

                // Insert the edge at the correct position.
                queue_.insert(it, keysAndEdge);
            }

            void ForwardQueue::insert(const std::vector<Edge> &edges)
            {
                // For now, let's just do this naively.
                for (const auto &edge : edges)
                {
                    insert(edge);
                }
            }

            void ForwardQueue::remove(const Edge &edge)
            {
                const auto it = std::find_if(queue_.begin(), queue_.end(), [&edge](const auto &keyEdgePair) {
                    return keyEdgePair.second.source->getId() == edge.source->getId() &&
                           keyEdgePair.second.target->getId() == edge.target->getId();
                });

                if (it != queue_.cend())
                {
                    // The forward queue does not have a lookup, just erase the edge.
                    queue_.erase(it);
                }
                else
                {
                    throw std::out_of_range("Can not remove edge from the forward queue, because it is not in the "
                                            "queue.");
                }
            }

            Edge ForwardQueue::peek(double suboptimalityFactor) const
            {
                // Make sure the queue contains edges.
                if (queue_.empty())
                {
                    throw std::out_of_range("Forward queue is empty, cannot peek.");
                }

                // Get the lower bounding edge and corresponding cost.
                const auto lowerBoundEdge = queue_.begin();
                const auto lowerBoundEdgeCost =
                    ompl::base::Cost(suboptimalityFactor * lowerBoundEdge->first.lowerBoundCost.value());

                // Find the best estimate edge and corresponding cost.
                const auto bestCostEdge = std::min_element(
                    queue_.begin(), queue_.end(),
                    [this](const std::pair<EdgeKeys, Edge> &a, const std::pair<EdgeKeys, Edge> &b) {
                        return objective_->isCostBetterThan(a.first.estimatedCost, b.first.estimatedCost);
                    });
                auto bestCostEdgeCost =
                    ompl::base::Cost(suboptimalityFactor * bestCostEdge->first.estimatedCost.value());

                // Find the least effort edge and corresponding cost.
                auto bestEffortEdgeCost = objective_->infiniteCost();
                auto bestEffortEdge = queue_.begin();
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
                    return bestEffortEdge->second;
                }
                else if (!objective_->isCostBetterThan(lowerBoundEdgeCost, bestCostEdge->first.estimatedCost))
                {
                    return bestCostEdge->second;
                }
                else
                {
                    return lowerBoundEdge->second;
                }
            }

            bool ForwardQueue::update(const Edge &edge)
            {
                // Compute the costs and effort.
                const auto lowerBoundCostOfEdge = lowerBoundCost(edge);
                const auto estimatedCostOfEdge = estimateCost(edge);
                const auto estimatedEffortOfEdge = estimateEffort(edge);

                const auto keysAndEdge = std::make_pair<EdgeKeys, Edge>(
                    {lowerBoundCostOfEdge, estimatedCostOfEdge, estimatedEffortOfEdge}, {edge.source, edge.target});

                // Check if the element already exists.
                const auto element =
                    std::find_if(queue_.cbegin(), queue_.cend(), [&edge](const std::pair<EdgeKeys, Edge> &element) {
                        return edge.source->getId() == element.second.source->getId() &&
                               edge.target->getId() == element.second.target->getId();
                    });

                // Find where to insert the edge.
                auto it = std::lower_bound(
                    queue_.cbegin(), queue_.cend(), keysAndEdge,
                    [this](const std::pair<EdgeKeys, Edge> &a, const std::pair<EdgeKeys, Edge> &b) {
                        return objective_->isCostBetterThan(a.first.lowerBoundCost, b.first.lowerBoundCost);
                    });

                // Check if the element exists.
                if (element != queue_.end())
                {
                    // If the element is already at the right place, simply return.
                    if (element == it)
                    {
                        return true;
                    }
                    else  // If the element exists but is not at the right place, erase it.
                    {
                        queue_.erase(element);
                    }
                    // Insert the edge at the correct position.
                    queue_.insert(it, keysAndEdge);
                }
                return false;
            }

            Edge ForwardQueue::pop(double suboptimalityFactor)
            {
                // Get the lower bounding edge and corresponding cost.
                const auto lowerBoundEdgeIt = queue_.cbegin();
                const auto lowerBoundEdgeCost =
                    ompl::base::Cost(suboptimalityFactor * lowerBoundEdgeIt->first.lowerBoundCost.value());

                // Find the best estimate edge and corresponding cost.
                const auto bestCostEdge = std::min_element(
                    queue_.cbegin(), queue_.cend(),
                    [this](const std::pair<EdgeKeys, Edge> &a, const std::pair<EdgeKeys, Edge> &b) {
                        return objective_->isCostBetterThan(a.first.estimatedCost, b.first.estimatedCost);
                    });
                const auto bestCostEdgeCost =
                    ompl::base::Cost(suboptimalityFactor * bestCostEdge->first.estimatedCost.value());

                // Find the least effort edge and corresponding cost.
                auto bestEffortEdgeCost = objective_->infiniteCost();
                auto bestEffortEdge = queue_.cbegin();
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
                    const auto bestEdge = bestEffortEdge->second;
                    queue_.erase(bestEffortEdge);
                    return bestEdge;
                }
                else if (!objective_->isCostBetterThan(lowerBoundEdgeCost, bestCostEdge->first.estimatedCost))
                {
                    const auto bestEdge = bestCostEdge->second;
                    queue_.erase(bestCostEdge);
                    return bestEdge;
                }
                else
                {
                    const auto lowerBoundEdge = lowerBoundEdgeIt->second;
                    queue_.erase(lowerBoundEdgeIt);
                    return lowerBoundEdge;
                }
            }

            ompl::base::Cost ForwardQueue::getLowerBoundOnOptimalSolutionCost() const
            {
                return queue_.begin()->first.lowerBoundCost;
            }

            void ForwardQueue::clear()
            {
                // Can simply call clear here, as we don't store pointers to edges in the forward queue.
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
                // Erase all elements that are not valid anymore.
                queue_.erase(std::remove_if(queue_.begin(), queue_.end(),
                                            [](const auto &keyEdgePair) {
                                                return !keyEdgePair.second.source->hasReverseVertex() ||
                                                       !keyEdgePair.second.target->hasReverseVertex();
                                            }),
                             queue_.end());

                // Update all remaining edges.
                for (const auto &element : queue_)
                {
                    update(element.second);
                }
            }

            std::size_t ForwardQueue::estimateEffort(const Edge &edge) const
            {
                return spaceInfo_->getStateSpace()->validSegmentCount(edge.source->raw(), edge.target->raw()) +
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

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

#include "ompl/geometric/planners/informedtrees/eitstar/ForwardQueue.h"

#include <algorithm>
#include <cmath>

#include "ompl/geometric/planners/informedtrees/eitstar/Direction.h"
#include "ompl/geometric/planners/informedtrees/eitstar/State.h"

namespace ompl
{
    namespace geometric
    {
        namespace eitstar
        {
            ForwardQueue::ForwardQueue(const std::shared_ptr<const ompl::base::OptimizationObjective> &objective,
                                       const std::shared_ptr<const ompl::base::StateSpace> &space)
              : objective_(objective), space_(space)
            {
                queue_.reserve(100u);
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
                modifiedQueue_ = true;

                // We update by removing the old edge and inserting the new.
                std::pair<std::size_t, std::size_t> key = std::make_pair(edge.source->getId(), edge.target->getId());
                const auto element = makeElement(edge);
                auto it = queue_.find(key);
                if (it != queue_.end())
                {
                    it->second = element;
                }
                else
                {
                    queue_.insert({key, element});
                }

                // Add it to the cache in the target state.
                edge.target->addToSourcesOfIncomingEdgesInForwardQueue(edge.source);
            }

            void ForwardQueue::insertOrUpdate(const std::vector<Edge> &edges)
            {
                modifiedQueue_ = true;

                // For now, let's just do this naively.
                for (const auto &edge : edges)
                {
                    insertOrUpdate(edge);
                }
            }

            void ForwardQueue::remove(const Edge &edge)
            {
                const std::pair<std::size_t, std::size_t> key =
                    std::make_pair(edge.source->getId(), edge.target->getId());
                const auto it = queue_.find(key);
                if (it != queue_.cend())
                {
                    // The forward queue does not have a lookup, just erase the edge.
                    queue_.erase(it);
                    edge.target->removeFromSourcesOfIncomingEdgesInForwardQueue(edge.source);

                    modifiedQueue_ = true;
                }
                else
                {
                    throw std::out_of_range("Can not remove edge from the forward queue, because it is not in the "
                                            "queue.");
                }
            }

            ForwardQueue::Container::const_iterator ForwardQueue::getFrontIter(double suboptimalityFactor)
            {
                if (cacheQueueLookup_ && !modifiedQueue_)
                {
                    return front_;
                }

                auto lowerBoundEdgeCost = objective_->infiniteCost();
                auto bestCostEdgeCost = objective_->infiniteCost();

                auto lowerBoundEdge = queue_.cbegin();
                auto bestCostEdge = queue_.cbegin();

                if (std::isfinite(suboptimalityFactor))
                {
                    // Get the lower bounding edge and corresponding cost.
                    lowerBoundEdge = getLowerBoundCostEdge();
                    lowerBoundEdgeCost = inflateCost(get(lowerBoundEdge).first.lowerBoundCost, suboptimalityFactor);

                    // Find the best estimate edge and corresponding cost.
                    bestCostEdge = getBestCostEstimateEdge();
                    bestCostEdgeCost = inflateCost(get(bestCostEdge).first.estimatedCost, suboptimalityFactor);
                }

                // Find the least effort edge and corresponding cost.
                auto bestEffortEdge = queue_.cbegin();
                auto bestEffortEdgeCost = objective_->infiniteCost();
                auto bestEffortLowerBoundCost = objective_->infiniteCost();
                for (auto it = queue_.cbegin(); it != queue_.cend(); ++it)
                {
                    if (get(it).first.estimatedEffort < get(bestEffortEdge).first.estimatedEffort &&
                        !objective_->isCostBetterThan(bestCostEdgeCost, get(it).first.estimatedCost))
                    {
                        bestEffortEdgeCost = get(it).first.estimatedCost;
                        bestEffortLowerBoundCost = get(it).first.lowerBoundCost;
                        bestEffortEdge = it;
                    }
                    else if (get(it).first.estimatedEffort == get(bestEffortEdge).first.estimatedEffort &&
                             !objective_->isCostBetterThan(bestCostEdgeCost, get(it).first.estimatedCost) &&
                             objective_->isCostBetterThan(get(it).first.lowerBoundCost, bestEffortLowerBoundCost))
                    {
                        bestEffortEdgeCost = get(it).first.estimatedCost;
                        bestEffortLowerBoundCost = get(it).first.lowerBoundCost;
                        bestEffortEdge = it;
                    }
                }

                // Return the correct edge.
                if (objective_->isCostBetterThan(bestEffortEdgeCost, lowerBoundEdgeCost))
                {
                    return bestEffortEdge;
                }
                else if (objective_->isCostBetterThan(get(bestCostEdge).first.estimatedCost, lowerBoundEdgeCost))
                {
                    return bestCostEdge;
                }
                else
                {
                    return lowerBoundEdge;
                }
            }

            Edge ForwardQueue::peek(double suboptimalityFactor)
            {
                // Make sure the queue contains edges.
                if (queue_.empty())
                {
                    throw std::out_of_range("Forward queue is empty, cannot peek.");
                }

                front_ = getFrontIter(suboptimalityFactor);
                auto edge = get(front_).second;

                // this call updates the cached minimum effort
                cachedMinEdgeEffort_ = getMinEffortToCome();
                modifiedQueue_ = false;

                return edge;
            }

            void ForwardQueue::updateIfExists(const Edge &edge)
            {
                // Get an iterator to the element in the queue.
                const std::pair<std::size_t, std::size_t> key =
                    std::make_pair(edge.source->getId(), edge.target->getId());
                const auto it = queue_.find(key);
                if (it == queue_.end())
                {
                    return;
                }

                it->second = makeElement(edge);
                modifiedQueue_ = true;
            }

            Edge ForwardQueue::pop(double suboptimalityFactor)
            {
                auto it = getFrontIter(suboptimalityFactor);

                const auto edge = get(it).second;
                queue_.erase(it);
                edge.target->removeFromSourcesOfIncomingEdgesInForwardQueue(edge.source);

                modifiedQueue_ = true;

                return edge;
            }

            unsigned int ForwardQueue::getMinEffortToCome() const
            {
                if (cacheQueueLookup_ && !modifiedQueue_)
                {
                    return cachedMinEdgeEffort_;
                }

                unsigned int minEdgeEffort = std::numeric_limits<unsigned int>::max();
                for (auto it = queue_.cbegin(); it != queue_.cend(); ++it)
                {
                    const auto edge = get(it).second;

                    if (edge.target->hasReverseVertex() || edge.target->hasForwardVertex())
                    {
                        continue;
                    }

                    if (edge.source->isWhitelisted(edge.target))
                    {
                        continue;
                    }

                    // Get the number of checks already performed on this edge.
                    const unsigned int performedChecks = edge.target->getIncomingCollisionCheckResolution(edge.source);
                    const unsigned int fullSegmentCount =
                        space_->validSegmentCount(edge.source->raw(), edge.target->raw());

                    const unsigned int edgeEffort = fullSegmentCount - performedChecks;

                    if (edgeEffort < minEdgeEffort)
                    {
                        minEdgeEffort = edgeEffort;
                    }

                    if (minEdgeEffort == 0u)
                    {
                        cachedMinEdgeEffort_ = 0;
                        return 0u;
                    }
                }

                cachedMinEdgeEffort_ = minEdgeEffort;
                return minEdgeEffort;
            }

            ompl::base::Cost ForwardQueue::getLowerBoundOnOptimalSolutionCost() const
            {
                auto it = getLowerBoundCostEdge();
                return get(it).first.lowerBoundCost;
            }

            bool ForwardQueue::containsOpenTargets(std::size_t reverseSearchTag) const
            {
                return std::find_if(queue_.begin(), queue_.end(),
                                    [reverseSearchTag](const auto &edge)
                                    {
                                        auto &a = edge.second;
                                        return a.second.target->asReverseVertex()->getExpandTag() != reverseSearchTag;
                                    }) != queue_.end();
            }

            void ForwardQueue::clear()
            {
                for (const auto &element : queue_)
                {
                    const auto &edge = element.second.second;
                    edge.target->resetSourcesOfIncomingEdgesInForwardQueue();
                }
                queue_.clear();
                modifiedQueue_ = true;
            }

            std::vector<Edge> ForwardQueue::getEdges() const
            {
                std::vector<Edge> edges;
                edges.reserve(queue_.size());
                for (const auto &element : queue_)
                {
                    const auto &edge = element.second.second;
                    edges.emplace_back(edge);
                }
                return edges;
            }

            void ForwardQueue::rebuild()
            {
                const auto edges = getEdges();
                clear();
                insertOrUpdate(edges);

                modifiedQueue_ = true;
            }

            std::pair<ForwardQueue::EdgeKeys, Edge> ForwardQueue::makeElement(const Edge &edge) const
            {
                // Compute the costs and effort.
                const auto lowerBoundCostOfEdge = lowerBoundCost(edge);
                const auto estimatedCostOfEdge = estimateCost(edge);
                const auto estimatedEffortOfEdge = estimateEffort(edge);

                return {{lowerBoundCostOfEdge, estimatedCostOfEdge, estimatedEffortOfEdge}, {edge.source, edge.target}};
            }

            ForwardQueue::Container::iterator ForwardQueue::getBestCostEstimateEdge()
            {
                // Find the best estimate edge and corresponding cost.
                return std::min_element(queue_.begin(), queue_.end(),
                                        [this](const auto &a, const auto &b)
                                        {
                                            const auto &aEdgeKey = a.second.first;
                                            const auto &bEdgeKey = b.second.first;
                                            return objective_->isCostBetterThan(aEdgeKey.estimatedCost,
                                                                                bEdgeKey.estimatedCost);
                                        });
            }

            ForwardQueue::Container::const_iterator ForwardQueue::getBestCostEstimateEdge() const
            {
                // Find the best estimate edge and corresponding cost.
                return std::min_element(queue_.cbegin(), queue_.cend(),
                                        [this](const auto &a, const auto &b)
                                        {
                                            const auto &aEdgeKey = a.second.first;
                                            const auto &bEdgeKey = b.second.first;
                                            return objective_->isCostBetterThan(aEdgeKey.estimatedCost,
                                                                                bEdgeKey.estimatedCost);
                                        });
            }

            ForwardQueue::Container::iterator ForwardQueue::getLowerBoundCostEdge()
            {
                auto it = std::max_element(queue_.begin(), queue_.end(),
                                           [this](const auto &a, const auto &b)
                                           {
                                               const auto &aEdgeKey = a.second.first;
                                               const auto &bEdgeKey = b.second.first;
                                               return !objective_->isCostBetterThan(aEdgeKey.lowerBoundCost,
                                                                                    bEdgeKey.lowerBoundCost);
                                           });

                return it;
            }

            ForwardQueue::Container::const_iterator ForwardQueue::getLowerBoundCostEdge() const
            {
                return std::max_element(queue_.cbegin(), queue_.cend(),
                                        [this](const auto &a, const auto &b)
                                        {
                                            const auto &aEdgeKey = a.second.first;
                                            const auto &bEdgeKey = b.second.first;
                                            return !objective_->isCostBetterThan(aEdgeKey.lowerBoundCost,
                                                                                 bEdgeKey.lowerBoundCost);
                                        });
            }

            ompl::base::Cost ForwardQueue::inflateCost(const ompl::base::Cost &cost, double factor) const
            {
                // Will this work with objectives that maximize cost (utility)?
                if (!std::isfinite(factor) || !objective_->isFinite(cost))
                {
                    return objective_->infiniteCost();
                }
                else
                {
                    return ompl::base::Cost(cost.value() * factor);
                }
            }

            std::size_t ForwardQueue::estimateEffort(const Edge &edge) const
            {
                unsigned int edgeEffort = 0u;
                if (edge.source->isWhitelisted(edge.target))
                {
                    edgeEffort = 0u;
                }
                else
                {
                    const unsigned int fullSegmentCount =
                        space_->validSegmentCount(edge.source->raw(), edge.target->raw());

                    // Get the number of checks already performed on this edge.
                    const unsigned int performedChecks = edge.target->getIncomingCollisionCheckResolution(edge.source);

                    edgeEffort = fullSegmentCount - performedChecks;
                }

                // Make sure this doesn't overflow.
                if (std::numeric_limits<unsigned int>::max() - edgeEffort < edge.target->getEstimatedEffortToGo())
                {
                    return std::numeric_limits<unsigned int>::max();
                }

                // Return the total effort
                return edge.target->getEstimatedEffortToGo() + edgeEffort;
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

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

#include "ompl/geometric/planners/aibitstar/ForwardQueue.h"

#include <algorithm>

#include "ompl/geometric/planners/aibitstar/Direction.h"
#include "ompl/geometric/planners/aibitstar/State.h"

namespace ompl
{
    namespace geometric
    {
        namespace aibitstar
        {
            ForwardQueue::ForwardQueue(const std::shared_ptr<const ompl::base::OptimizationObjective> &objective,
                                       const std::shared_ptr<const ompl::base::SpaceInformation> &spaceInfo)
              : objective_(objective), spaceInfo_(spaceInfo), queue_()
            {
                queue_.reserve(10000u);
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
                // Check if the element already exists.
                auto element = std::find_if(queue_.begin(), queue_.end(), [&edge](const Edge &element) {
                    return edge.source->getId() == element.source->getId() &&
                           edge.target->getId() == element.target->getId();
                });

                // If it exists, update the queue, otherwise insert it at the right place.
                if (element != queue_.end())
                {
                    std::sort(queue_.begin(), queue_.end(), [this](const Edge &a, const Edge &b) {
                        return objective_->isCostBetterThan(lowerBoundCost(a), lowerBoundCost(b));
                    });
                }
                else
                {
                    // Find where to insert the edge.
                    auto it =
                        std::lower_bound(queue_.begin(), queue_.end(), edge, [this](const Edge &a, const Edge &b) {
                            return objective_->isCostBetterThan(lowerBoundCost(a), lowerBoundCost(b));
                        });

                    // Insert the edge at the correct position.
                    queue_.insert(it, edge);
                }
            }

            void ForwardQueue::insert(const std::vector<Edge> &edges)
            {
                // For now, let's just do this naively.
                for (const auto &edge : edges)
                {
                    insert(edge);
                }
            }

            Edge ForwardQueue::peek(float suboptimalityFactor) const
            {
                // Get the lower bounding edge and corresponding cost.
                Edge lowerBoundEdge = queue_.front();
                auto lowerBoundEdgeCost =
                    ompl::base::Cost(suboptimalityFactor * lowerBoundCost(lowerBoundEdge).value());

                // Find the best estimate edge and corresponding cost.
                auto bestCostEdge =
                    std::min_element(queue_.begin(), queue_.end(), [this](const Edge &a, const Edge &b) {
                        return objective_->isCostBetterThan(estimateCost(a), estimateCost(b));
                    });
                auto bestCostEdgeCost = ompl::base::Cost(suboptimalityFactor * estimateCost(*bestCostEdge).value());

                // Find the least effort edge and corresponding cost.
                std::size_t bestEffort = std::numeric_limits<std::size_t>::max();
                auto bestEffortEdgeCost = objective_->infiniteCost();
                auto bestEffortEdge = queue_.begin();
                for (auto it = queue_.begin(); it != queue_.end(); ++it)
                {
                    // Estimate the effort for this edge.
                    auto effortEstimate = estimateEffort(*it);

                    // Estimate the cost for this edge.
                    auto costEstimate = estimateCost(*it);

                    if (effortEstimate < bestEffort && !objective_->isCostBetterThan(bestCostEdgeCost, costEstimate))
                    {
                        bestEffort = effortEstimate;
                        bestEffortEdge = it;
                        bestEffortEdgeCost = costEstimate;
                    }
                }

                // Return the correct edge.
                if (!objective_->isCostBetterThan(lowerBoundEdgeCost, bestEffortEdgeCost))
                {
                    return *bestEffortEdge;
                }
                else if (!objective_->isCostBetterThan(lowerBoundEdgeCost, estimateCost(*bestCostEdge)))
                {
                    return *bestCostEdge;
                }
                else
                {
                    return lowerBoundEdge;
                }
            }

            bool ForwardQueue::update(const Edge &edge)
            {
                // Check if the element already exists.
                auto element = std::find_if(queue_.begin(), queue_.end(), [&edge](const Edge &element) {
                    return edge.source->getId() == element.source->getId() &&
                           edge.target->getId() == element.target->getId();
                });

                // If it exists, update the queue, otherwise insert it at the right place.
                if (element != queue_.end())
                {
                    std::sort(queue_.begin(), queue_.end(), [this](const Edge &a, const Edge &b) {
                        return objective_->isCostBetterThan(lowerBoundCost(a), lowerBoundCost(b));
                    });
                    return true;
                }
                else
                {
                    return false;
                }
            }

            Edge ForwardQueue::pop(float suboptimalityFactor)
            {
                // Get the lower bounding edge and corresponding cost.
                Edge lowerBoundEdge = queue_.front();
                auto lowerBoundEdgeCost =
                    ompl::base::Cost(suboptimalityFactor * lowerBoundCost(lowerBoundEdge).value());

                // Find the best estimate edge and corresponding cost.
                auto bestCostEdge =
                    std::min_element(queue_.begin(), queue_.end(), [this](const Edge &a, const Edge &b) {
                        return objective_->isCostBetterThan(estimateCost(a), estimateCost(b));
                    });
                auto bestCostEdgeCost = ompl::base::Cost(suboptimalityFactor * estimateCost(*bestCostEdge).value());

                // Find the least effort edge and corresponding cost.
                std::size_t bestEffort = std::numeric_limits<std::size_t>::max();
                auto bestEffortEdgeCost = objective_->infiniteCost();
                auto bestEffortEdge = queue_.begin();
                for (auto it = queue_.begin(); it != queue_.end(); ++it)
                {
                    // Estimate the effort for this edge.
                    auto effortEstimate = estimateEffort(*it);

                    // Estimate the cost for this edge.
                    auto costEstimate = estimateCost(*it);

                    if (effortEstimate < bestEffort && !objective_->isCostBetterThan(bestCostEdgeCost, costEstimate))
                    {
                        bestEffort = effortEstimate;
                        bestEffortEdge = it;
                        bestEffortEdgeCost = costEstimate;
                    }
                }

                // Return the correct edge.
                if (!objective_->isCostBetterThan(lowerBoundEdgeCost, bestEffortEdgeCost))
                {
                    auto bestEdge = *bestEffortEdge;
                    queue_.erase(bestEffortEdge);
                    return bestEdge;
                }
                else if (!objective_->isCostBetterThan(lowerBoundEdgeCost, estimateCost(*bestCostEdge)))
                {
                    auto bestEdge = *bestCostEdge;
                    queue_.erase(bestCostEdge);
                    return bestEdge;
                }
                else
                {
                    queue_.erase(queue_.begin());
                    return lowerBoundEdge;
                }
            }

            ompl::base::Cost ForwardQueue::getLowerBoundOnOptimalSolutionCost() const
            {
                return lowerBoundCost(queue_.front());
            }

            void ForwardQueue::clear()
            {
                queue_.clear();
            }

            std::vector<Edge> ForwardQueue::getEdges() const
            {
                return queue_;
            }

            std::size_t ForwardQueue::estimateEffort(const Edge &edge) const
            {
                return spaceInfo_->getStateSpace()->validSegmentCount(edge.source->raw(), edge.target->raw()) +
                       edge.target->getEstimatedEffortToGo();
            }

            ompl::base::Cost ForwardQueue::estimateCost(const Edge &edge) const
            {
                return objective_->combineCosts(objective_->combineCosts(edge.source->asForwardVertex()->getCost(),
                                                                         objective_->motionCostBestEstimate(
                                                                             edge.source->raw(), edge.target->raw())),
                                                edge.target->getEstimatedCostToGo());
            }

            ompl::base::Cost ForwardQueue::lowerBoundCost(const Edge &edge) const
            {
                return objective_->combineCosts(
                    objective_->combineCosts(edge.source->asForwardVertex()->getCost(),
                                             objective_->motionCostHeuristic(edge.source->raw(), edge.target->raw())),
                    edge.target->getLowerBoundCostToGo());
            }

        }  // namespace aibitstar

    }  // namespace geometric

}  // namespace ompl

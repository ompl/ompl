/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, University of New Hampshire
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
 *   * Neither the names of the copyright holders nor the names of its
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

// Authors: Yi Wang, Eyal Weiss, Bingxian Mu, Oren Salzman
#include "ompl/geometric/planners/lazyinformedtrees/blitstar/Vertex.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <string>

#include "ompl/base/goals/GoalState.h"
#include "ompl/base/goals/GoalStates.h"
using namespace std;
using namespace std::string_literals;

namespace ompl
{
    namespace geometric
    {
        namespace blitstar
        {
            namespace
            {
std::size_t generateId()
                {
                    static std::atomic<std::size_t> id{0u};
                    return id++;
                }
            }  // namespace

            Vertex::Vertex(const ompl::base::SpaceInformationPtr &spaceInformation,
                           const ompl::base::ProblemDefinitionPtr &problemDefinition, const std::size_t &batchId)
              : spaceInformation_(spaceInformation)
              , problemDefinition_(problemDefinition)
              , objective_(problemDefinition->getOptimizationObjective())
              , forwardChildren_()
              , reverseChildren_()
              , forwardParent_()
              , reverseParent_()
              , state_(spaceInformation->allocState())  // The memory allocated here is freed in the destructor.
              , costToComeFromStart_(objective_->infiniteCost())
              , costToComeFromGoal_(objective_->infiniteCost())
              , edgeCostFromForwardParent_(objective_->infiniteCost())
              , expandedCostToComeFromGoal_(objective_->infiniteCost())
              , costToGoToGoal_(objective_->infiniteCost())
              , costToGoToStart_(objective_->infiniteCost())
              , vertexId_(generateId())
              , batchId_(batchId)
            {
            }

            Vertex::Vertex(const std::shared_ptr<Vertex> &other)
              : spaceInformation_(other->spaceInformation_)
              , problemDefinition_(other->problemDefinition_)
              , objective_(other->objective_)
              , forwardChildren_(other->forwardChildren_)
              , reverseChildren_(other->reverseChildren_)
              , forwardParent_(other->forwardParent_)
              , reverseParent_(other->reverseParent_)
              , state_(spaceInformation_->allocState())  // The memory allocated here is freed in the destructor.
              , costToComeFromStart_(other->costToComeFromStart_)
              , costToComeFromGoal_(other->costToComeFromGoal_)
              , edgeCostFromForwardParent_(other->edgeCostFromForwardParent_)
              , expandedCostToComeFromGoal_(other->expandedCostToComeFromGoal_)
              , costToGoToGoal_(other->costToGoToGoal_)
              , costToGoToStart_(other->costToGoToStart_)
              , vertexId_(other->vertexId_)
              , batchId_(other->batchId_)
            {
                spaceInformation_->copyState(state_, other->getState());
            }

            Vertex::~Vertex()
            {
                // The state has associated memory that needs to be freed manually.
                spaceInformation_->freeState(state_);
            };

            std::size_t Vertex::getId() const
            {
                return vertexId_;
            }

            ompl::base::State *Vertex::getState()
            {
                return state_;
            }

            ompl::base::State const *Vertex::getState() const
            {
                return state_;
            }

            ompl::base::ScopedState<> Vertex::getScopedState() const
            {
                return ompl::base::ScopedState<>(spaceInformation_->getStateSpace(), state_);
            }

            ompl::base::Cost Vertex::getCostToComeFromStart() const
            {
                return costToComeFromStart_;
            }         
            
            
            bool Vertex::isGoal()
            {
                return  goalVertex_;
            }
            
            bool Vertex::isStart()
            {
                return  startVertex_; 
            } 
            void Vertex::setGoalVertex()
            {
                  goalVertex_ = true; 
            }
            void Vertex::setStartVertex()
            {
                  startVertex_ = true;
            }
                       
            void Vertex::setForwardId(const std::size_t counter)
            {
                    ForwardVersion_ = counter;
            }
            std::size_t Vertex::getForwardId()
            {
                   return ForwardVersion_; 
            }
            void Vertex::resetForwardId()
            {
                   ForwardVersion_ = 0u;
            }
            
            void Vertex::setReverseId(const std::size_t counter)
            {
                    ReverseVersion_ = counter;
            }
            std::size_t Vertex::getReverseId()
            {
                   return ReverseVersion_; 
            }
            void Vertex::resetReverseId()
            {
                   ReverseVersion_ = 0u;
            }

            ompl::base::Cost Vertex::getCostToComeFromGoal() const
            {
                return costToComeFromGoal_;
            }
            
            ompl::base::Cost Vertex::getCostToGoToGoal() const
            {
                return getCostToComeFromGoal();
            }

            ompl::base::Cost Vertex::getEdgeCostFromForwardParent() const
            {
                return edgeCostFromForwardParent_;
            }
            
            ompl::base::Cost Vertex::getValidForwardEdgeCost() const
            {
                return edgeCostFromValidForwardParent_;
            }
            
            ompl::base::Cost Vertex::getValidReverseEdgeCost() const
            {
                return edgeCostFromValidReverseParent_;
            } 
            
            ompl::base::Cost Vertex::getEdgeCostFromReverseParent() const
            {
                return edgeCostFromBackwardParent_;
            }            
  
            bool Vertex::hasForwardParent() const
            {
                return static_cast<bool>(forwardParent_.lock());
            }

            bool Vertex::hasForwardEdgeParent() const
            {
                return static_cast<bool>(forwardEdgeParent_.lock());
            }
            
            std::shared_ptr<Vertex> Vertex::getForwardParent() const
            {
                return forwardParent_.lock();
            }

            std::shared_ptr<Vertex> Vertex::getForwardEdgeParent() const
            {
                return forwardEdgeParent_.lock();
            }


            bool Vertex::hasReverseParent() const
            {
                return static_cast<bool>(reverseParent_.lock());
            }

            bool Vertex::hasReverseEdgeParent() const
            {
                return static_cast<bool>(reverseEdgeParent_.lock());
            }

            std::shared_ptr<Vertex> Vertex::getReverseParent() const
            {
                return reverseParent_.lock();
            }

            std::shared_ptr<Vertex> Vertex::getReverseEdgeParent() const
            {
                return reverseEdgeParent_.lock();
            }

            void Vertex::setForwardEdgeCost(const ompl::base::Cost &cost)
            {
                edgeCostFromForwardParent_ = cost;
            }

            void Vertex::setCostToComeFromStart(const ompl::base::Cost &cost)
            {
                costToComeFromStart_ = cost;
            }            
            
            void Vertex::resetMeet()
            {
                meetVertex_ = false;
            }
            void Vertex::setMeet()
            {
                meetVertex_ = true;
            }
            bool Vertex::meetVertex()
            {
                return meetVertex_;
            }
            
            void Vertex::setCostToComeFromGoal(const ompl::base::Cost &cost)
            {
                costToComeFromGoal_ = cost;
            }
             
            void Vertex::resetCostToComeFromGoal()
            {
                costToComeFromGoal_ = objective_->infiniteCost();
            }

            void Vertex::resetCostToComeFromStart()
            {
                costToComeFromStart_ = objective_->infiniteCost();
            }

            void Vertex::setForwardInvalid()
            {
                  forwardInvalidChildState_ = true;
            }
            
            bool Vertex::forwardInvalid()
            {
                  return forwardInvalidChildState_;
            }
            
            void Vertex::resetForwardInvalid()
            {
                  forwardInvalidChildState_ = false;
            }
            
            void Vertex::setReverseInvalid()
            {
                  reverseInvalidChildState_ = true; 
            }
             
            bool Vertex::reverseInvalid()
            {
                  return reverseInvalidChildState_;
            }
            
            void Vertex::resetReverseInvalid()
            {
                  reverseInvalidChildState_ = false;  
            }
            
            void Vertex::setCostToGoToGoal(const ompl::base::Cost &cost)
            {
                costToGoToGoal_ = cost;
            }

            void Vertex::setCostToGoToStart(const ompl::base::Cost &cost)
            {
                costToGoToStart_ = cost;
            }            

            void Vertex::setNearObstacle() 
            {
                nearObstacle_ = true;
            }

            bool Vertex::nearObstacle()
            {
                   return nearObstacle_; 
            }

            void Vertex::resetNearObstacle() {
                  nearObstacle_ = false;
            }
            bool Vertex::isForwardExpanded()
            {
                return IsForwardExpanded_ ;
            }
            
            void Vertex::setForwardExpanded()
            {
                IsForwardExpanded_ = true;
            }
            
            void Vertex::resetForwardExpanded()
            {
                IsForwardExpanded_ = false;
            }
            
            bool Vertex::isReverseExpanded()
            {
                return IsReverseExpanded_ ;
            }
            
            void Vertex::setReverseExpanded()
            {
                IsReverseExpanded_ = true;
            }
            
            void Vertex::resetReverseExpanded()
            {
                IsReverseExpanded_ = false;
            }               
            
            void Vertex::setLowerCostBoundToStart(const ompl::base::Cost &ToStart)
            {
                  lowerCostBoundToGoToStart_ = ToStart;      
            }             

            void Vertex::setLowerCostBoundToGoal(const ompl::base::Cost &ToGoal)
            {
                  lowerCostBoundToGoToGoal_ = ToGoal;  
            }   
              
            ompl::base::Cost Vertex::getLowerCostBoundToStart()
            {
                  return lowerCostBoundToGoToStart_;      
            }             

            ompl::base::Cost Vertex::getLowerCostBoundToGoal()
            {
                  return lowerCostBoundToGoToGoal_;  
            }

            void Vertex::setForwardValidParent(const std::shared_ptr<Vertex> &vertex, const ompl::base::Cost &edgeCost)
            {
                // Remember the edge cost.
                edgeCostFromValidForwardParent_ = edgeCost;
                // Remember the corresponding parent.
                forwardEdgeParent_ = std::weak_ptr<Vertex>(vertex); 
            }

            void Vertex::setReverseValidParent(const std::shared_ptr<Vertex> &vertex, const ompl::base::Cost &edgeCost)
            {
                // Remember the edge cost.
                edgeCostFromValidReverseParent_ = edgeCost;
                // Remember the corresponding parent.
                reverseEdgeParent_ = std::weak_ptr<Vertex>(vertex);    
            }   

            void Vertex::resetForwardParent()
            {
                forwardParent_.reset();
            }
           
            void Vertex::resetForwardEdgeParent()
            {
                forwardEdgeParent_.reset();
            }

            void Vertex::setForwardVertexParent(const std::shared_ptr<Vertex> &vertex, const ompl::base::Cost &edgeCost)
            {
                // If this is a rewiring, remove from my parent's children.
                if(static_cast<bool>(forwardParent_.lock()))//&&(!static_cast<bool>(forwardEdgeParent_.lock()) ||  forwardParent_.lock()->getId() != forwardEdgeParent_.lock()->getId())
                {   
                       forwardParent_.lock()->removeFromForwardChildren(vertexId_);
                }
                // Remember the parent.
                edgeCostFromForwardParent_ = edgeCost;
                forwardParent_ = std::weak_ptr<Vertex>(vertex);
            }

            void Vertex::setReverseVertexParent(const std::shared_ptr<Vertex> &vertex, const ompl::base::Cost &edgeCost)
            {  
                // If this is a rewiring, remove from my parent's children.
                if (static_cast<bool>(reverseParent_.lock()))//(!static_cast<bool>(reverseEdgeParent_.lock()) || reverseParent_.lock()->getId() != reverseEdgeParent_.lock()->getId())
                {  
                       reverseParent_.lock()->removeFromReverseChildren(vertexId_);
                }
                // Remember the parent.
                edgeCostFromBackwardParent_ = edgeCost;
                reverseParent_ = std::weak_ptr<Vertex>(vertex);
            }

            void Vertex::resetReverseParent()
            {
                reverseParent_.reset();
            }

            void Vertex::resetReverseEdgeParent()
            {
                reverseEdgeParent_.reset();
            }

            void Vertex::addToForwardChildren(const std::shared_ptr<Vertex> &vertex)
            {
                forwardChildren_.emplace_back(vertex);
            }

            void Vertex::removeFromForwardChildren(std::size_t vertexId)
            {
                // Find the child.
                auto it = std::find_if(
                    forwardChildren_.begin(), forwardChildren_.end(),
                    [vertexId](const std::weak_ptr<Vertex> &child) { return vertexId == child.lock()->getId(); });
                
                // Throw if it is not found.
                if (it == forwardChildren_.end())
                {
                    auto msg = "Asked to remove vertex from forward children that is currently not a child."s;
                    throw ompl::Exception(msg);
                }
                // Swap and pop.
                
                std::iter_swap(it, forwardChildren_.rbegin());
                forwardChildren_.pop_back();
            }
            
            void Vertex::addToReverseChildren(const std::shared_ptr<Vertex> &vertex)
            {
                reverseChildren_.push_back(vertex);
            }

            void Vertex::removeFromReverseChildren(std::size_t vertexId)
            {
                // Find the child.
                auto it = std::find_if(
                    reverseChildren_.begin(), reverseChildren_.end(),
                    [vertexId](const std::weak_ptr<Vertex> &child) {return vertexId == child.lock()->getId(); });

                // Throw if it is not found.
                if (it == reverseChildren_.end())
                {
                    auto msg = "Asked to remove vertex from reverse children that is currently not a child."s;
                    throw ompl::Exception(msg);
                }

                // Swap and pop.
                std::iter_swap(it, reverseChildren_.rbegin());
                reverseChildren_.pop_back();
            }

            void Vertex::whitelistAsChild(const std::shared_ptr<Vertex> &vertex) const
            {
                whitelistedChildren_.emplace_back(vertex);
            }

            bool Vertex::isWhitelistedAsChild(const std::shared_ptr<Vertex> &vertex) const
            {
                // Check if the vertex is whitelisted by iterating over all whitelisted children.
                // It this detects an invalid vertex, e.g., a vertex that was once whitelisted but
                // has been pruned since, remove the vertex from the list of whitelisted children.
                auto it = whitelistedChildren_.begin();
                while (it != whitelistedChildren_.end())
                {
                    // Check if the child is a valid vertex.
                    if (const auto child = it->lock())
                    {
                        // Check if the vertex is whitelisted.
                        if (child->getId() == vertex->getId())
                        {
                            return true;
                        }
                        ++it;
                    }
                    else
                    {
                        it = whitelistedChildren_.erase(it);
                    }
                }
                return false;
            }
             
            void Vertex::setIncomingCollisionCheckResolution(const std::size_t vertexId,
                                                            std::size_t numChecks) const
            {
                incomingCollisionCheckResolution_[vertexId] = numChecks;
            }

            std::size_t Vertex::getIncomingCollisionCheckResolution(const std::size_t vertexId) const
            {
                if (incomingCollisionCheckResolution_.find(vertexId) == incomingCollisionCheckResolution_.end())
                {
                    return 0u;
                }
                else
                {
                    return incomingCollisionCheckResolution_[vertexId];
                }
            } 
            
            void Vertex::blacklistAsChild(const std::shared_ptr<Vertex> &vertex) const
            {
                blacklistedChildren_.emplace_back(vertex);
            }

            bool Vertex::isBlacklistedAsChild(const std::shared_ptr<Vertex> &vertex) const
            {
                auto it = blacklistedChildren_.begin();
                while (it != blacklistedChildren_.end())
                {
                    // Check if the child is a valid vertex.
                    if (const auto child = it->lock())
                    {
                        // Check if the vertex is whitelisted.
                        if (child->getId() == vertex->getId())
                        {
                            return true;
                        }
                        ++it;
                    }
                    else
                    {
                        it = blacklistedChildren_.erase(it);
                    }
                }
                return false;
            }

            bool Vertex::hasCachedNeighbors() const
            {
                return neighborBatchId_ == batchId_;
            }

            void Vertex::cacheNeighbors(const std::vector<std::shared_ptr<Vertex>> &neighbors) const
            {
                neighbors_.clear();
                neighbors_.insert(neighbors_.begin(), neighbors.begin(), neighbors.end());
                neighborBatchId_ = batchId_;
            }

            const std::vector<std::shared_ptr<Vertex>> Vertex::getNeighbors() const
            {
                if (neighborBatchId_ != batchId_)
                {
                    throw ompl::Exception("Requested neighbors from vertex of outdated approximation.");
                }

                std::vector<std::shared_ptr<Vertex>> neighbors;
                for (const auto &neighbor : neighbors_)
                {
                    assert(neighbor.lock());
                    neighbors.emplace_back(neighbor.lock());
                }

                return neighbors;
            }

            std::vector<std::shared_ptr<Vertex>> Vertex::getForwardChildren() const
            {
                std::vector<std::shared_ptr<Vertex>> children;
                for (const auto &child : forwardChildren_)
                {
                    assert(!child.expired());
                    children.emplace_back(child.lock());
                }
                return children;
            }

            std::vector<std::shared_ptr<Vertex>> Vertex::getReverseChildren() const
            {
                std::vector<std::shared_ptr<Vertex>> children;
                children.reserve(reverseChildren_.size());
                for (const auto &child : reverseChildren_)
                {
                    assert(!child.expired());
                    children.emplace_back(child.lock());
                }
                return children;
            }

            void Vertex::setReverseVertexQueuePointer(
                typename ompl::BinaryHeap<
                    std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>,
                    std::function<bool(const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &,
                                       const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &)>>::
                    Element *pointer)
            {
                reverseVertexQueuePointerId_ = batchId_;
                reverseVertexQueuePointer_ = pointer;
            }

            void Vertex::setForwardVertexQueuePointer(
                typename ompl::BinaryHeap<
                    std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>,
                    std::function<bool(const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &,
                                       const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &)>>::
                    Element *pointer)
            {
                forwardVertexQueuePointerId_ = batchId_;
                forwardVertexQueuePointer_ = pointer;
            }

            typename ompl::BinaryHeap<
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>,
                std::function<bool(const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &,
                                   const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &)>>::
                Element *
                Vertex::getForwardVertexQueuePointer() const
            {
                if (batchId_ != forwardVertexQueuePointerId_)
                {
                    forwardVertexQueuePointer_ = nullptr;
                }
                return forwardVertexQueuePointer_;
            }


            typename ompl::BinaryHeap<
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>,
                std::function<bool(const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &,
                                   const std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> &)>>::
                Element *
                Vertex::getReverseVertexQueuePointer() const
            {
                if (batchId_ != reverseVertexQueuePointerId_)
                {
                    reverseVertexQueuePointer_ = nullptr;
                }
                return reverseVertexQueuePointer_;
            }          

            void Vertex::resetForwardVertexQueuePointer()
            {
                forwardVertexQueuePointer_ = nullptr;
            }
            
            void Vertex::resetReverseVertexQueuePointer()
            {
                reverseVertexQueuePointer_ = nullptr;
            }

        }  // namespace blitstar

    }  // namespace geometric

}  // namespace ompl

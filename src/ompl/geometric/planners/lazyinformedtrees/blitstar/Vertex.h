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
 *
 *********************************************************************/
/**********************************************************************
 * Attribution Notice:
 *
 * This file contains code partially derived from the AIT* or BIT* planner
 * in the Open Motion Planning Library (OMPL). Structural elements such as
 * vertex representation and basic parent–child relationships were adapted
 * from that planner.
 *
 * Additional modifications and new logic for the BLIT* planner were
 * independently developed by Yi Wang.
 *********************************************************************/


// Authors: Yi Wang, Eyal Weiss, Bingxian Mu, Oren Salzman

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BLITSTAR_VERTEX_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BLITSTAR_VERTEX_

#include <memory>
#include <vector>

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/State.h"
#include "ompl/datastructures/BinaryHeap.h"

#include "ompl/geometric/planners/lazyinformedtrees/blitstar/Queuetypes.h"

namespace ompl
{
    namespace geometric
    {
        namespace blitstar
        {
            class Vertex : public std::enable_shared_from_this<Vertex>
            {
                    public:
                /** \brief Constructs a vertex by sampling a state. */
                Vertex(const ompl::base::SpaceInformationPtr &spaceInformation,
                       const ompl::base::ProblemDefinitionPtr &problemDefinition, const std::size_t &batchId);

                /** \brief Constructs a copy of another vertex. */
                explicit Vertex(const std::shared_ptr<Vertex> &other);

                /** \brief Destructs the vertex. */
                virtual ~Vertex();

                /** \brief Get the unique id of this vertex. */
                std::size_t getId() const;

                /** \brief Provides write access to the underlying state. */
                ompl::base::State *getState();

                /** \brief Provides read access to the underlying state. */
                ompl::base::State const *getState() const;

                /** \brief Returns a scoped copy of the underlying state. */
                ompl::base::ScopedState<> getScopedState() const;

                /** \brief Returns the cost to come to this vertex from the start. */
                ompl::base::Cost getCostToComeFromStart() const;

                /** \brief Returns the cost to come to this vertex from the goal. */
                ompl::base::Cost getCostToComeFromGoal() const;

                /** \brief Returns the cost to go heuristic from this vertex. */
                ompl::base::Cost getCostToGoToGoal() const;

                /** \brief Returns the edge cost from the forward parent. */
                ompl::base::Cost getEdgeCostFromForwardParent() const;
                
                /** \brief Returns the edge cost from the reverse parent. */
                ompl::base::Cost getEdgeCostFromReverseParent() const;
                
                /** \brief Returns the valid edge cost from the forward and backward tree on a valid path. */
                ompl::base::Cost getValidForwardEdgeCost() const;  
                ompl::base::Cost getValidReverseEdgeCost() const;
                
                /** \brief Resets associated parents of this vertex. */
                void resetForwardParent();
                void resetReverseParent();
                void resetForwardEdgeParent();
                void resetReverseEdgeParent();
                
                /** \brief Returns whether this vertex has a parent in either search. */
                bool hasForwardParent() const;
                bool hasReverseParent() const;
                bool hasReverseEdgeParent() const;
                bool hasForwardEdgeParent() const;

                /** \brief Returns the parent of the vertex (in the forward-search tree). */
                std::shared_ptr<Vertex> getForwardParent() const;
                std::shared_ptr<Vertex> getForwardEdgeParent() const;
                
                /** \brief Returns the parent of the vertex (in the reverse-search tree). */
                std::shared_ptr<Vertex> getReverseParent() const;
                std::shared_ptr<Vertex> getReverseEdgeParent() const;
                
                /** \brief Sets the cost to come to this vertex. */
                void setForwardEdgeCost(const ompl::base::Cost &cost);

                /** \brief Sets the cost to come to this vertex. */
                void setCostToComeFromStart(const ompl::base::Cost &cost);        

                /** \brief Sets the cost to come to this vertex from the goal. */
                void setCostToComeFromGoal(const ompl::base::Cost &cost);

                /** \brief Sets the cost to go To goal heuristic of this vertex. */
                void setCostToGoToGoal(const ompl::base::Cost &cost);

                /** \brief Sets the cost to go To start heuristic of this vertex. */
                void setCostToGoToStart(const ompl::base::Cost &cost);

                /** \brief Adds a vertex to this vertex's forward children. */
                void addToForwardChildren(const std::shared_ptr<Vertex> &vertex);

                /** \brief Removes a vertex from this vertex's forward children. */
                void removeFromForwardChildren(std::size_t vertexId);

                /** \brief Returns this vertex's children in the forward search tree. */
                std::vector<std::shared_ptr<Vertex>> getForwardChildren() const;

                /** \brief Adds a vertex this vertex's children. */
                void addToReverseChildren(const std::shared_ptr<Vertex> &vertex);

                /** \brief Removes a vertex from this vertex's forward children. */
                void removeFromReverseChildren(std::size_t vertexId);

                /** \brief Returns this vertex's children in the reverse search tree. */
                std::vector<std::shared_ptr<Vertex>> getReverseChildren() const;

                /** \brief Whitelists a child. */
                void whitelistAsChild(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Returns whether a child is whitelisted. */
                bool isWhitelistedAsChild(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Blacklists a child. */
                void blacklistAsChild(const std::shared_ptr<Vertex> &vertex) const;


                std::size_t getIncomingCollisionCheckResolution(const std::size_t vertexId) const;

                void setIncomingCollisionCheckResolution(const std::size_t vertexId, std::size_t numChecks) const;   
                /** \brief Returns whether a child is blacklisted. */
                bool isBlacklistedAsChild(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Returns whether the vertex knows its nearest neighbors on the current approximation. */
                bool hasCachedNeighbors() const;  
                /** \brief Caches the neighbors for the current approximation. */
                void cacheNeighbors(const std::vector<std::shared_ptr<Vertex>> &neighbors) const;

                /** \brief Returns the nearest neighbors, throws if not up to date. */
                const std::vector<std::shared_ptr<Vertex>> getNeighbors() const;

                /** \brief set and evalue whether this vertex is a start, goal or meeting state. */
                bool isGoal();
                void setMeet(); 
                bool isStart(); 
                bool meetVertex();
                
                /** \brief whether a state is near obstalce*/
                bool nearObstacle();
                void setGoalVertex();
                bool forwardInvalid();
                bool reverseInvalid();
                
                /** \brief remark the state to be near obstalce. */
                void setStartVertex();
                void setNearObstacle();
                void setForwardInvalid();
                void setReverseInvalid();
                
                /** \brief Sets wether this vertex is expanded. */
                void setReverseExpanded();
                void setForwardExpanded();
                
                /** \brief Check whether a state is expanded. */
                bool isForwardExpanded();
                bool isReverseExpanded();
                
                /** \brief get the current search counter of a state*/
                std::size_t getForwardId();
                std::size_t getReverseId();
                
                /** \brief Set the the lower bound of the estimated heuristic value*/
                ompl::base::Cost getLowerCostBoundToGoal();
                ompl::base::Cost getLowerCostBoundToStart();
                
                /** \brief Set the current search counter*/
                void setForwardId(const std::size_t counter);
                void setReverseId(const std::size_t counter);
                
                /** \brief Set the lower-cost-bound-to-go of this vertex. */
                void setLowerCostBoundToGoal(const ompl::base::Cost & ToGoal);
                void setLowerCostBoundToStart(const ompl::base::Cost & ToStart);
                
                /** \brief Resets the value of aforementioned values.*/
                void resetMeet();
                void resetNearObstacle(); 
                void resetBackwardParent();
                void resetForwardId();
                void resetReverseId();
                void resetForwardInvalid();
                void resetReverseInvalid();
                void resetForwardExpanded();
                void resetReverseExpanded();
                void resetCostToComeFromGoal();
                void resetCostToComeFromStart();
                
                /** \brief Resets the reverse queue pointer. */
                void resetForwardVertexQueuePointer();
                void resetReverseVertexQueuePointer();

                /** \brief Returns the reverse queue pointer of this vertex. */
                typename VertexQueue::Element *getForwardVertexQueuePointer() const;
                typename VertexQueue::Element *getReverseVertexQueuePointer() const;
                /** \brief Sets the reverse queue pointer of this vertex. */
                void setForwardVertexQueuePointer(typename VertexQueue::Element *pointer);
                void setReverseVertexQueuePointer(typename VertexQueue::Element *pointer);
                
                /** \brief Sets the valid parent vertex (in a valid path). */
                void setForwardValidParent(const std::shared_ptr<Vertex> &vertex, const ompl::base::Cost &edgeCost); 
                void setReverseValidParent(const std::shared_ptr<Vertex> &vertex, const ompl::base::Cost &edgeCost);
                                                
                /** \brief Sets the parent vertex (in the reverse or forward -search tree). */
                void setReverseVertexParent(const std::shared_ptr<Vertex> &vertex,const ompl::base::Cost &edgeCost);
                void setForwardVertexParent(const std::shared_ptr<Vertex> &vertex,const ompl::base::Cost &edgeCost);
                

            private:
                /** \brief The space information of the planning problem. */
                const ompl::base::SpaceInformationPtr spaceInformation_;

                /** \brief The definition of the planning problem. */
                const ompl::base::ProblemDefinitionPtr problemDefinition_;

                /** \brief The optimization objective of the planning problem. */
                const ompl::base::OptimizationObjectivePtr objective_;

                /** \brief The children of this vertex in the forward and reverse search trees. */
                std::vector<std::weak_ptr<Vertex>> forwardChildren_{};
                std::vector<std::weak_ptr<Vertex>> reverseChildren_{};
                
                /** \brief The cached neighbors of this vertex. */
                mutable std::vector<std::weak_ptr<Vertex>> neighbors_{};

                /** \brief The list of whitelisted children. */
                mutable std::vector<std::weak_ptr<Vertex>> whitelistedChildren_{};

                /** \brief The list of blacklisted children. */
                mutable std::vector<std::weak_ptr<Vertex>> blacklistedChildren_{};

                /** \brief The parent of this vertex in the forward search tree. */
                std::weak_ptr<Vertex> forwardParent_;
                std::weak_ptr<Vertex> forwardEdgeParent_; 
                
                /** \brief The parent of this vertex in the reverse search tree. */
                std::weak_ptr<Vertex> reverseParent_;
                std::weak_ptr<Vertex> reverseEdgeParent_; 
                
                /** \brief The state associated with this vertex. */
                ompl::base::State *state_;

                /** \brief The cost to come from start/goal/middler to this vertex. */
                mutable ompl::base::Cost costToComeFromStart_{0u};
                mutable ompl::base::Cost costToComeFromGoal_{0u};

                /** \brief Lower cost bound to go to goal/start, such as eculidean distance. */
                ompl::base::Cost lowerCostBoundToGoToGoal_{0u};
                ompl::base::Cost lowerCostBoundToGoToStart_{0u};

                /** \brief The edge cost from the parent in each direction. */
                ompl::base::Cost edgeCostFromForwardParent_{0u};
                ompl::base::Cost edgeCostFromBackwardParent_{0u};
                
                /** \brief The edge cost from the parent in a valid path. */
                ompl::base::Cost edgeCostFromValidForwardParent_{0u};
                ompl::base::Cost edgeCostFromValidReverseParent_{0u};

                /** \brief The cost to come from the goal when this vertex was expanded. */
                mutable ompl::base::Cost expandedCostToComeFromGoal_{0u};
                mutable ompl::base::Cost expandedCostToComeFromStart_{0u};
                
                /** \brief The cost to go to Goal for estimating for this vertex(Admissible estimated heuristic to goal)*/
                mutable ompl::base::Cost costToGoToGoal_{0u};

                /** \brief The cost to go to start for estimating for this vertex.(Admissible estimated heuristic to start)*/
                mutable ompl::base::Cost costToGoToStart_{0u};
                
                /** \brief The total estimated solution for this vertex (Admissible estimated solution cost)*/
                mutable ompl::base::Cost totalEstimateCost_{0u};
                
                /** \brief whether the vertex is start, goal, or the current meeting state*/
                bool goalVertex_{false};
                bool meetVertex_{false};
                bool startVertex_{false};  
                
                /** \brief Whether the vertex is near obstacle-region*/
                bool nearObstacle_{false};
                  
                /** \brief The unique id of this vertex. */
                const std::size_t vertexId_; 
                
                /** \brief The id of the most recent batch. */
                const std::size_t &batchId_; 
                 
                /** \brief whether the vertex is already expanded in either search direction*/
                bool IsForwardExpanded_{false};
                bool IsReverseExpanded_{false};
                
                /** \brief A tag assigned to each restart of the search process. */
                std::size_t ForwardVersion_{0};
                std::size_t ReverseVersion_{0};
                
                /** \brief whether the state is the child of an invalid edge in either search direction*/
                bool forwardInvalidChildState_{false};
                bool reverseInvalidChildState_{false};

                /** \brief The batch id for which the cached neighbor list is valid. */
                mutable std::size_t neighborBatchId_{0u};
                
                /** \brief The forward and reverse search ids for which the queue pointers are valid. */
                mutable std::size_t forwardVertexQueuePointerId_{0u};
                mutable std::size_t reverseVertexQueuePointerId_{0u};
                
                mutable std::map<std::size_t, std::size_t> incomingCollisionCheckResolution_{};
                /** \brief The pointers to the forward and reverse queue element. */
                mutable typename VertexQueue::Element *reverseVertexQueuePointer_{nullptr};
                mutable typename VertexQueue::Element *forwardVertexQueuePointer_{nullptr};
            };

        }  // namespace blitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BLITSTAR_VERTEX_

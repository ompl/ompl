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

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EITSTAR_VERTEX_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EITSTAR_VERTEX_

#include <memory>
#include <vector>

#include "ompl/base/Cost.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/BinaryHeap.h"

#include "ompl/geometric/planners/informedtrees/eitstar/Direction.h"
#include "ompl/geometric/planners/informedtrees/eitstar/Edge.h"

namespace ompl
{
    namespace geometric
    {
        namespace eitstar
        {
            // Forward declare the EIT* state class.
            class State;

            /** \brief The vertex class for both the forward and reverse search. */
            class Vertex : public std::enable_shared_from_this<Vertex>  // Inheritance must be public here.
            {
            public:
                /** \brief Constructs the vertex, which must be associated with a state. */
                Vertex(const std::shared_ptr<State> &state,
                       const std::shared_ptr<ompl::base::OptimizationObjective> &objective, const Direction &direction);

                /** \brief Destructs this vertex. */
                ~Vertex();

                /** \brief Gets the unique vertex-id of this vertex. */
                std::size_t getId() const;

                /** \brief Returns the state associated with this vertex. */
                std::shared_ptr<State> getState() const;

                /** \brief Returns the children of this vertex. */
                const std::vector<std::shared_ptr<Vertex>> &getChildren() const;

                /** \brief Returns whether this vertex has children. */
                bool hasChildren() const;

                /** \brief Update the cost-to-come of this vertex's children. */
                std::vector<std::shared_ptr<Vertex>>
                updateCurrentCostOfChildren(const std::shared_ptr<ompl::base::OptimizationObjective> &objective);

                /** \brief Adds the given vertex to this vertex's children. */
                void addChild(const std::shared_ptr<Vertex> &vertex);

                /** \brief Removes the given vertex from this vertex's children. */
                void removeChild(const std::shared_ptr<Vertex> &vertex);

                /** \brief Returns the parent of this vertex. */
                std::weak_ptr<Vertex> getParent() const;

                /** \brief Returns whether the given vertex is this vertex's parent. */
                bool isParent(const std::shared_ptr<Vertex> &vertex) const;

                /** \brief Returns the twin of this vertex, i.e., the vertex in the other search tree with the same
                 * underlying state. */
                std::weak_ptr<Vertex> getTwin() const;

                /** \brief Resets the parent of this vertex. */
                void updateParent(const std::shared_ptr<Vertex> &vertex);

                /** \brief Sets the cost of the edge in the forward tree that leads to this vertex. */
                void setEdgeCost(const ompl::base::Cost &edgeCost);

                /** \brief Returns the cost of the edge in the forward tree that leads to this vertex. */
                ompl::base::Cost getEdgeCost() const;

                /** \brief Returns the parent of this vertex. */
                void resetParent();

                /** \brief Sets the twin of this vertex, i.e., the vertex in the other search tree with the same
                 * underlying state. */
                void setTwin(const std::shared_ptr<Vertex> &vertex);

                /** \brief Resets the children of this vertex. */
                void clearChildren();

                /** \brief Returns the tag when this vertex was last expanded. */
                std::size_t getExpandTag() const;

                /** \brief Sets the expand tag when this vertex was last expanded. */
                void registerExpansionInReverseSearch(std::size_t tag);

                /** \brief Recursively calls the given function on this vertex and all its children in the tree. */
                void callOnBranch(const std::function<void(const std::shared_ptr<eitstar::State> &)> &function);

            private:
                /** \brief The unique id of this vertex. */
                const std::size_t id_;

                /** \brief The objective. */
                const std::shared_ptr<const ompl::base::OptimizationObjective> objective_;

                /** \brief The cost of the edge which connects this vertex with its parent. */
                ompl::base::Cost edgeCost_{std::numeric_limits<double>::signaling_NaN()};

                /** \brief The parent of this vertex. */
                std::weak_ptr<Vertex> parent_{};

                /** \brief The twin of this vertex, i.e., the vertex with the same underlying state in the other search
                 * tree. */
                std::weak_ptr<Vertex> twin_{};

                /** \brief The children of this vertex. */
                std::vector<std::shared_ptr<Vertex>> children_{};

                /** \brief The tag when this vertex was last expanded. */
                std::size_t expandTag_{0u};

                /** \brief The state this vertex is associated with. */
                const std::shared_ptr<State> state_;

                /** \brief The direction of the tree this vertex is part of. */
                const Direction direction_;

                /** \brief The edge queue is a friend class to allow efficient updates of outgoing edges of this vertex
                 * in the queue. */
                friend class ReverseQueue;

                /** \brief The outgoing edges from this vertex currently in the queue. This is maintained by the queue.
                 */
                mutable std::vector<ompl::BinaryHeap<
                    std::tuple<ompl::base::Cost, ompl::base::Cost, unsigned int, unsigned int, Edge>,
                    std::function<bool(
                        const std::tuple<ompl::base::Cost, ompl::base::Cost, unsigned int, unsigned int, Edge> &,
                        const std::tuple<ompl::base::Cost, ompl::base::Cost, unsigned int, unsigned int, Edge> &)>>::
                                        Element *>
                    outgoingReverseQueueLookup_;
            };

        }  // namespace eitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EITSTAR_QUEUE_

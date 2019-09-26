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

#include "ompl/geometric/planners/aibitstar/Direction.h"

namespace ompl
{
    namespace geometric
    {
        namespace aibitstar
        {
            template <Direction D>
            EdgeQueue<D>::EdgeQueue() : heap_([](const Edge &lhs, const Edge &rhs) { return lhs.key < rhs.key; })
            {
            }

            template <Direction D>
            bool EdgeQueue<D>::empty() const
            {
                return size() == 0u;
            }

            template <Direction D>
            std::size_t EdgeQueue<D>::size() const
            {
                return heap_.size();
            }

            template <>
            void EdgeQueue<Direction::FORWARD>::insert(const Edge& edge);

            template <>
            void EdgeQueue<Direction::REVERSE>::insert(const Edge& edge);

            template <Direction D>
            void EdgeQueue<D>::insert(const Edge &edge)
            {
                // static_assert(false, ...) is never satisfied, even if only specializations are instantiated.
                static_assert(D == Direction::FORWARD || D == Direction::REVERSE, "The edge queue must be instantiated "
                                                                                  "with Direction::FORWARD or "
                                                                                  "Direction::REVERSE as template "
                                                                                  "parameter.");
            }

            template <Direction D>
            void EdgeQueue<D>::insert(const std::vector<Edge> &edges)
            {
                // Can't use the heap's method for multiple edges because we need the packward pointers.
                for (const auto &edge : edges)
                {
                    insert(edge);
                }
            }

            template <Direction D>
            const Edge &EdgeQueue<D>::peek() const
            {
                // Return a reference to the top element in the queue, otherwise throw.
                if (auto element = heap_.top())
                {
                    return element->data;
                }
                else
                {
                    throw std::out_of_range("There are no elements in the queue.");
                }
            }

            template<>
            Edge EdgeQueue<Direction::FORWARD>::pop();

            template<>
            Edge EdgeQueue<Direction::REVERSE>::pop();

            template <Direction D>
            Edge EdgeQueue<D>::pop()
            {
                // static_assert(false, ...) is never satisfied, even if only specializations are instantiated.
                static_assert(D == Direction::FORWARD || D == Direction::REVERSE, "The edge queue must be instantiated "
                                                                                  "with Direction::FORWARD or "
                                                                                  "Direction::REVERSE as template "
                                                                                  "parameter.");
                return Edge();
            }

            template <Direction D>
            void EdgeQueue<D>::clear()
            {
                // Can't use heap_.clear() here, because we need to remove the outgoing edge queue lookup pointers here.
                while (!empty())
                {
                    auto edge = pop();
                }
            }

            template <Direction D>
            std::vector<Edge> EdgeQueue<D>::getEdges() const
            {
                std::vector<Edge> edges;
                heap_.getContent(edges);
                return edges;
            }

            template <Direction D>
            void EdgeQueue<D>::rebuild()
            {
                heap_.rebuild();
            }

        }  // namespace aibitstar

    }  // namespace geometric

}  // namespace ompl

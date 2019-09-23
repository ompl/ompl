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

#include "ompl/geometric/planners/aibitstar/Queue.h"

namespace ompl
{
    namespace geometric
    {
        namespace aibitstar
        {
            EdgeQueue::EdgeQueue() : heap_([](const Edge &lhs, const Edge &rhs) { return lhs.key < rhs.key; })
            {
            }

            bool EdgeQueue::empty() const
            {
                return size() == 0u;
            }

            std::size_t EdgeQueue::size() const
            {
                return heap_.size();
            }

            void EdgeQueue::insert(const Edge &edge)
            {
                // Simply update if it is already in the queue.
                for (const auto outgoingEdge : edge.parent->outgoingEdgeQueueLookup_)
                {
                    if (outgoingEdge->data.child->getId() == edge.child->getId())
                    {
                        if (edge.key < outgoingEdge->data.key)
                        {
                            outgoingEdge->data.key = edge.key;
                            heap_.update(outgoingEdge);
                        }
                        return;
                    }
                }

                // Insert the edge and emplace the resulting data pointer into the parents outgoing edge lookup.
                edge.parent->outgoingEdgeQueueLookup_.emplace_back(heap_.insert(edge));
            }

            void EdgeQueue::insert(const std::vector<Edge> &edges)
            {
                // Can't use the heap's method for multiple edges because we need the packward pointers.
                for (const auto &edge : edges)
                {
                    insert(edge);
                }
            }

            const Edge &EdgeQueue::peek() const
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

            Edge EdgeQueue::pop()
            {
                // Get the top element in the queue, throw if empty.
                if (auto element = heap_.top())
                {
                    // Copy the top element of the heap.
                    Edge top = element->data;

                    // Pop the element from the heap.
                    heap_.pop();

                    // Get a reference to the outgoing edge queue lookup of the parent vertex.
                    auto &outgoingEdgeQueueLookup = top.parent->outgoingEdgeQueueLookup_;

                    // Remove the edge from the outgoing edge lookup of the parent vertex using find, swap and pop.
                    auto it = std::find(outgoingEdgeQueueLookup.begin(), outgoingEdgeQueueLookup.end(), element);

                    // If this edge is not in the lookup, this is a bug.
                    assert(it != outgoingEdgeQueueLookup.end());

                    // Do the good ol' swappedy poppy.
                    std::iter_swap(it, outgoingEdgeQueueLookup.rbegin());
                    outgoingEdgeQueueLookup.pop_back();

                    // Return the top element.
                    return top;
                }
                else
                {
                    throw std::out_of_range("There are no elements in the queue.");
                }
            }

            void EdgeQueue::clear()
            {
                // Starting to wonder whether the edge queue lookup is actually useful.
                while (!empty())
                {
                    pop();
                }
            }

            std::vector<Edge> EdgeQueue::getEdges() const
            {
                std::vector<Edge> edges;
                heap_.getContent(edges);
                return edges;
            }

            void EdgeQueue::rebuild()
            {
                heap_.rebuild();
            }

        }  // namespace aibitstar

    }  // namespace geometric

}  // namespace ompl

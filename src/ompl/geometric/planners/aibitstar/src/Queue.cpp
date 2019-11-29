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

#include "ompl/geometric/planners/aibitstar/Direction.h"
#include "ompl/geometric/planners/aibitstar/State.h"

namespace ompl
{
    namespace geometric
    {
        namespace aibitstar
        {
            template <>
            void EdgeQueue<Direction::FORWARD>::insert(const Edge &edge)
            {
                // Update if the edge is already in the queue.
                if(!update(edge)) {
                    edge.source->asForwardVertex()->outgoingEdgeQueueLookup_.emplace_back(heap_.insert(edge));
                }
            }

            template <>
            void EdgeQueue<Direction::REVERSE>::insert(const Edge &edge)
            {
                // Update if the edge is already in the queue.
                if (!update(edge)) {
                    edge.source->asReverseVertex()->outgoingEdgeQueueLookup_.emplace_back(heap_.insert(edge));
                }
            }

            template <>
            bool EdgeQueue<Direction::FORWARD>::update(const Edge &edge)
            {
                // Update if the edge is already in the queue.
                for (const auto outgoingEdge : edge.source->asForwardVertex()->outgoingEdgeQueueLookup_)
                {
                    if (outgoingEdge->data.target->getId() == edge.target->getId())
                    {
                        if (edge.key < outgoingEdge->data.key)
                        {
                            outgoingEdge->data.key = edge.key;
                            heap_.update(outgoingEdge);
                        }
                        return true;
                    }
                }
                // throw std::runtime_error("Could not update edge, because it is not in the outgoing queue lookup.");
                return false;
            }

            template <>
            bool EdgeQueue<Direction::REVERSE>::update(const Edge &edge)
            {
                for (const auto outgoingEdge : edge.source->asReverseVertex()->outgoingEdgeQueueLookup_)
                {
                    if (outgoingEdge->data.target->getId() == edge.target->getId())
                    {
                        if (edge.key < outgoingEdge->data.key)
                        {
                            outgoingEdge->data.key = edge.key;
                            heap_.update(outgoingEdge);
                        }
                        return true;
                    }
                }
                return false;
            }

            template <>
            Edge EdgeQueue<Direction::FORWARD>::pop()
            {
                // Get the top element in the queue, throw if empty.
                if (auto element = heap_.top())
                {
                    // Copy the top element of the heap.
                    Edge top = element->data;

                    // Pop the element from the heap.
                    heap_.pop();

                    // Get a reference to the outgoing edge queue lookup of the source vertex.
                    auto &outgoingEdgeQueueLookup = top.source->asForwardVertex()->outgoingEdgeQueueLookup_;

                    // Remove the edge from the ougoing edge lookup of the source vertex using find, swap and pop.
                    auto it = std::find(outgoingEdgeQueueLookup.begin(), outgoingEdgeQueueLookup.end(), element);

                    // If this edge is not in the lookup, this is a bug.
                    assert(it != outgoingEdgeQueueLookup.end());

                    // Do the good ol' swappedy poppedy.
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

            template <>
            Edge EdgeQueue<Direction::REVERSE>::pop()
            {
                // Get the top element in the queue, throw if empty.
                if (auto element = heap_.top())
                {
                    // Copy the top element of the heap.
                    Edge top = element->data;

                    // Pop the element from the heap.
                    heap_.pop();

                    // Get a reference to the outgoing edge queue lookup of the source vertex.
                    auto &outgoingEdgeQueueLookup = top.source->asReverseVertex()->outgoingEdgeQueueLookup_;

                    // Remove the edge from the ougoing edge lookup of the source vertex using find, swap and pop.
                    auto it = std::find(outgoingEdgeQueueLookup.begin(), outgoingEdgeQueueLookup.end(), element);

                    // If this edge is not in the lookup, this is a bug.
                    assert(it != outgoingEdgeQueueLookup.end());

                    // Do the good ol' swappedy poppedy.
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

        }  // namespace aibitstar

    }  // namespace geometric

}  // namespace ompl

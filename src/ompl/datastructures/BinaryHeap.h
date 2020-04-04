/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#ifndef OMPL_DATASTRUCTURES_BINARY_HEAP_
#define OMPL_DATASTRUCTURES_BINARY_HEAP_

#include <functional>
#include <utility>
#include <vector>
#include <cassert>

namespace ompl
{
    /** \brief This class provides an implementation of an updatable
        min-heap. Using it is a bit cumbersome, as it requires keeping
        track of the BinaryHeap::Element* type, however, it should be
        as fast as it gets with an updatable heap. */
    template <typename _T, class LessThan = std::less<_T>>
    class BinaryHeap
    {
    public:
        /** \brief When an element is added to the heap, an instance
            of Element* is created. This instance contains the data
            that was added and internal information about the position
            of the data in the heap's internal storage. */
        class Element
        {
            friend class BinaryHeap;

        private:
            Element() = default;
            ~Element() = default;
            /** \brief The location of the data in the heap's storage */
            unsigned int position;

        public:
            /** \brief The data of this element */
            _T data;
        };

        /** \brief Event that gets called after an insertion */
        using EventAfterInsert = void (*)(Element *, void *);

        /** \brief Event that gets called just before a removal */
        using EventBeforeRemove = void (*)(Element *, void *);

        BinaryHeap()
        {
            eventAfterInsert_ = nullptr;
            eventBeforeRemove_ = nullptr;
        }

        BinaryHeap(LessThan lt) : lt_(std::move(lt))
        {
            eventAfterInsert_ = nullptr;
            eventBeforeRemove_ = nullptr;
        }

        ~BinaryHeap()
        {
            clear();
        }

        /** \brief Set the event that gets called after insertion */
        void onAfterInsert(EventAfterInsert event, void *arg)
        {
            eventAfterInsert_ = event;
            eventAfterInsertData_ = arg;
        }

        /** \brief Set the event that gets called before a removal */
        void onBeforeRemove(EventBeforeRemove event, void *arg)
        {
            eventBeforeRemove_ = event;
            eventBeforeRemoveData_ = arg;
        }

        /** \brief Clear the heap */
        void clear()
        {
            for (auto &element : vector_)
                delete element;
            vector_.clear();
        }

        /** \brief Return the top element. nullptr for an empty heap. */
        Element *top() const
        {
            return vector_.empty() ? nullptr : vector_.at(0);
        }

        /** \brief Remove the top element */
        void pop()
        {
            removePos(0);
        }

        /** \brief Remove a specific element */
        void remove(Element *element)
        {
            if (eventBeforeRemove_)
                eventBeforeRemove_(element, eventBeforeRemoveData_);
            removePos(element->position);
        }

        /** \brief Add a new element */
        Element *insert(const _T &data)
        {
            auto *element = new Element();
            element->data = data;
            const unsigned int pos = vector_.size();
            element->position = pos;
            vector_.push_back(element);
            percolateUp(pos);
            if (eventAfterInsert_)
                eventAfterInsert_(element, eventAfterInsertData_);
            return element;
        }

        /** \brief Add a set of elements to the heap */
        void insert(const std::vector<_T> &list)
        {
            const unsigned int n = vector_.size();
            const unsigned int m = list.size();
            for (unsigned int i = 0; i < m; ++i)
            {
                const unsigned int pos = i + n;
                Element *element = newElement(list[i], pos);
                vector_.push_back(element);
                percolateUp(pos);
                if (eventAfterInsert_)
                    eventAfterInsert_(element, eventAfterInsertData_);
            }
        }

        /** \brief Clear the heap, add the set of elements @e list to it and rebuild it. */
        void buildFrom(const std::vector<_T> &list)
        {
            clear();
            const unsigned int m = list.size();
            for (unsigned int i = 0; i < m; ++i)
                vector_.push_back(newElement(list[i], i));
            build();
        }

        /** \brief Rebuild the heap */
        void rebuild()
        {
            build();
        }

        /** \brief Update an element in the heap */
        void update(Element *element)
        {
            const unsigned int pos = element->position;
            assert(vector_[pos] == element);
            percolateUp(pos);
            percolateDown(pos);
        }

        /** \brief Check if the heap is empty */
        bool empty() const
        {
            return vector_.empty();
        }

        /** \brief Get the number of elements in the heap */
        unsigned int size() const
        {
            return vector_.size();
        }

        /** \brief Get the data stored in this heap */
        void getContent(std::vector<_T> &content) const
        {
            for (auto &element : vector_)
                content.push_back(element->data);
        }

        /** \brief Sort an array of elements. This does not affect the content of the heap */
        void sort(std::vector<_T> &list)
        {
            const unsigned int n = list.size();
            std::vector<Element *> backup = vector_;
            vector_.clear();
            for (unsigned int i = 0; i < n; ++i)
                vector_.push_back(newElement(list[i], i));
            build();
            list.clear();
            list.reserve(n);

            for (unsigned int i = 0; i < n; ++i)
            {
                list.push_back(vector_[0]->data);
                removePos(0);
            }
            vector_ = backup;
        }

        /** \brief Return a reference to the comparison operator */
        LessThan &getComparisonOperator()
        {
            return lt_;
        }

    private:
        LessThan lt_;

        std::vector<Element *> vector_;

        EventAfterInsert eventAfterInsert_;
        void *eventAfterInsertData_;
        EventBeforeRemove eventBeforeRemove_;
        void *eventBeforeRemoveData_;

        void removePos(unsigned int pos)
        {
            const int n = vector_.size() - 1;
            delete vector_[pos];
            if ((int)pos < n)
            {
                vector_[pos] = vector_.back();
                vector_[pos]->position = pos;
                vector_.pop_back();
                percolateDown(pos);
            }
            else
                vector_.pop_back();
        }

        Element *newElement(const _T &data, unsigned int pos) const
        {
            auto *element = new Element();
            element->data = data;
            element->position = pos;
            return element;
        }

        void build()
        {
            for (int i = vector_.size() / 2 - 1; i >= 0; --i)
                percolateDown(i);
        }

        void percolateDown(const unsigned int pos)
        {
            const unsigned int n = vector_.size();
            Element *tmp = vector_[pos];
            unsigned int parent = pos;
            unsigned int child = (pos + 1) << 1;

            while (child < n)
            {
                if (lt_(vector_[child - 1]->data, vector_[child]->data))
                    --child;
                if (lt_(vector_[child]->data, tmp->data))
                {
                    vector_[parent] = vector_[child];
                    vector_[parent]->position = parent;
                }
                else
                    break;
                parent = child;
                child = (child + 1) << 1;
            }
            if (child == n)
            {
                --child;
                if (lt_(vector_[child]->data, tmp->data))
                {
                    vector_[parent] = vector_[child];
                    vector_[parent]->position = parent;
                    parent = child;
                }
            }
            if (parent != pos)
            {
                vector_[parent] = tmp;
                vector_[parent]->position = parent;
            }
        }

        void percolateUp(const unsigned int pos)
        {
            Element *tmp = vector_[pos];
            unsigned int child = pos;
            unsigned int parent = (pos - 1) >> 1;

            while (child > 0 && lt_(tmp->data, vector_[parent]->data))
            {
                vector_[child] = vector_[parent];
                vector_[child]->position = child;
                child = parent;
                parent = (parent - 1) >> 1;
            }
            if (child != pos)
            {
                vector_[child] = tmp;
                vector_[child]->position = child;
            }
        }
    };
}

#endif

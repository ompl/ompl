/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Matt Maly */

#ifndef OMPL_DATASTRUCTURES_PDF_
#define OMPL_DATASTRUCTURES_PDF_

#include "ompl/util/Exception.h"
#include <ostream>
#include <vector>

namespace ompl
{
    template <typename _T>
    class PDF
    {
    public:

        class Element
        {
            friend class PDF;
        public:
            _T data_;
        private:
            Element(const _T& d, const std::size_t i) : data_(d), index_(i)
            {
            }
            std::size_t index_;
        };

        PDF(void)
        {
        }

        PDF(const std::vector<_T>& d, const std::vector<double>& weights)
        {
            if (d.size() != weights.size())
                throw Exception("Data vector and weight vector must be of equal length");
            //by default, reserve space for 512 elements
            data_.reserve(512u);
            //n elements require at most log2(n)+2 rows of the tree
            tree_.reserve(11u);
            for (std::size_t i = 0; i < d.size(); ++i)
                add(d[i], weights[i]);
        }

        ~PDF(void)
        {
            clear();
        }

        Element& add(const _T& d, const double w)
        {
            if (w < 0)
                throw Exception("Weight argument must be a nonnegative value");
            Element* elem = new Element(d, data_.size());
            data_.push_back(elem);
            if (data_.size() == 1)
            {
                std::vector<double> r(1, w);
                tree_.push_back(r);
                return *elem;
            }
            tree_.front().push_back(w);
            for (std::size_t i = 1; i < tree_.size(); ++i)
            {
                if (tree_[i-1].size() % 2 == 1)
                    tree_[i].push_back(w);
                else
                {
                    while (i < tree_.size())
                    {
                        tree_[i].back() += w;
                        ++i;
                    }
                    return *elem;
                }
            }
            //If we've made it here, then we need to add a new head to the tree.
            std::vector<double> head(1, tree_.back()[0] + tree_.back()[1]);
            tree_.push_back(head);
            return *elem;
        }

        const _T& sample(double r) const
        {
            if (data_.empty())
                throw Exception("Cannot sample from an empty PDF");
            if (r < 0 || r > 1)
                throw Exception("Sampling value must be between 0 and 1");
            std::size_t row = tree_.size() - 1;
            r *= tree_[row].front();
            std::size_t node = 0;
            while (row != 0)
            {
                --row;
                node <<= 1;
                if (r > tree_[row][node])
                {
                    r -= tree_[row][node];
                    ++node;
                }
            }
            return data_[node]->data_;
        }

        void update(Element& elem, const double w)
        {
            std::size_t index = elem.index_;
            if (index >= data_.size())
                throw Exception("Element to update is not in PDF");
            const double weightChange = w - tree_.front()[index];
            tree_.front()[index] = w;
            index >>= 1;
            for (std::size_t row = 1; row < tree_.size(); ++row)
            {
                tree_[row][index] += weightChange;
                index >>= 1;
            }
        }

        void remove(Element& elem)
        {
            if (data_.size() == 1)
            {
                delete data_.front();
                data_.clear();
                tree_.clear();
                return;
            }

            const std::size_t index = elem.index_;
            delete data_[index];
            std::swap(data_[index], data_.back());
            data_[index]->index_ = index;
            std::swap(tree_.front()[index], tree_.front().back());

            double weight;
            /* If index and back() are siblings in the tree, then
             * we don't need to make an extra pass over the tree.
             * The amount by which we change the values at the edge
             * of the tree is different in this case. */
            if (index+2 == data_.size() && index%2 == 0)
                weight = tree_.front().back();
            else
            {
                weight = tree_.front()[index];
                const double weightChange = weight - tree_.front().back();
                std::size_t parent = index >> 1;
                for (std::size_t row = 1; row < tree_.size(); ++row)
                {
                    tree_[row][parent] += weightChange;
                    parent >>= 1;
                }
            }

            /* Now that the element to remove is at the edge of the tree,
             * pop it off and update the corresponding weights. */
            data_.pop_back();
            tree_.front().pop_back();
            for (std::size_t i = 1; i < tree_.size() && tree_[i-1].size() > 1; ++i)
            {
                if (tree_[i-1].size() % 2 == 0)
                    tree_[i].pop_back();
                else
                {
                    while (i < tree_.size())
                    {
                        tree_[i].back() -= weight;
                        ++i;
                    }
                    return;
                }
            }
            //If we've made it here, then we need to remove a redundant head from the tree.
            tree_.pop_back();
        }

        void clear(void)
        {
            for (typename std::vector<Element*>::iterator e = data_.begin(); e != data_.end(); ++e)
                delete *e;
            data_.clear();
            tree_.clear();
        }

        std::size_t size(void) const
        {
            return data_.size();
        }

        bool empty(void) const
        {
            return data_.empty();
        }

        void printTree(std::ostream& out = std::cout) const
        {
            if (tree_.empty())
                return;
            for (std::size_t j = 0; j < tree_[0].size(); ++j)
                out << "(" << data_[j]->data << "," << tree_[0][j] << ") ";
            out << std::endl;
            for (std::size_t i = 1; i < tree_.size(); ++i)
            {
                for (std::size_t j = 0; j < tree_[i].size(); ++j)
                    out << tree_[i][j] << " ";
                out << std::endl;
            }
            out << std::endl;
        }

    private:

        std::vector<Element*> data_;
        std::vector<std::vector<double > > tree_;
    };
}

#endif

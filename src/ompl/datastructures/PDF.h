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
            _T data;
            private:
            Element(const _T& d, const std::size_t i) : data(d), index(i)
            {
            }
            std::size_t index;
        };

        PDF(void)
        {
        }

        PDF(const std::vector<_T>& d, const std::vector<double>& weights)
        {
            if (d.size() != weights.size())
                throw Exception("Data vector and weight vector must be of equal length");

            //n elements of data require at most (log2(n)+2) rows in tree
            std::size_t pow = 2;
            std::size_t lg = 0;
            while (pow <= d.size())
            {
                ++lg;
                pow <<= 1;
            }
            data.reserve(d.size());
            tree.reserve(lg + 2);
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
            Element* elem = new Element(d, data.size());
            data.push_back(elem);
            if (data.size() == 1)
            {
                std::vector<double> r(1, w);
                tree.push_back(r);
                return *elem;
            }
            tree.front().push_back(w);
            for (std::size_t i = 1; i < tree.size(); ++i)
            {
                if (tree[i-1].size() % 2 == 1)
                    tree[i].push_back(w);
                else
                {
                    while (i < tree.size())
                    {
                        tree[i].back() += w;
                        ++i;
                    }
                    return *elem;
                }
            }
            //If we've made it here, then we need to add a new head to the tree.
            std::vector<double> head(1, tree.back()[0] + tree.back()[1]);
            tree.push_back(head);
            return *elem;
        }

        const _T& sample(double r) const
        {
            if (data.empty())
                throw Exception("Cannot sample from an empty PDF");
            if (r < 0 || r > 1)
                throw Exception("Sampling value must be between 0 and 1");
            std::size_t row = tree.size() - 1;
            r *= tree[row].front();
            std::size_t node = 0;
            while (row != 0)
            {
                --row;
                node <<= 1;
                if (r > tree[row][node])
                {
                    r -= tree[row][node];
                    ++node;
                }
            }
            return data[node]->data;
        }

        void update(Element& elem, const double w)
        {
            std::size_t index = elem.index;
            if (index >= data.size())
                throw Exception("Element to update is not in PDF");
            const double weightChange = w - tree.front()[index];
            tree.front()[index] = w;
            index >>= 1;
            for (std::size_t row = 1; row < tree.size(); ++row)
            {
                tree[row][index] += weightChange;
                index >>= 1;
            }
        }

        void remove(Element& elem)
        {
            if (data.size() == 1)
            {
                delete data.front();
                data.clear();
                tree.clear();
                return;
            }

            const std::size_t index = elem.index;
            delete data[index];
            std::swap(data[index], data.back());
            data[index]->index = index;
            std::swap(tree.front()[index], tree.front().back());

            double weight;
            /* If index and back() are siblings in the tree, then
             * we don't need to make an extra pass over the tree.
             * The amount by which we change the values at the edge
             * of the tree is different in this case. */
            if (index+2 == data.size() && index%2 == 0)
                weight = tree.front().back();
            else
            {
                weight = tree.front()[index];
                const double weightChange = weight - tree.front().back();
                std::size_t parent = index >> 1;
                for (std::size_t row = 1; row < tree.size(); ++row)
                {
                    tree[row][parent] += weightChange;
                    parent >>= 1;
                }
            }

            /* Now that the element to remove is at the edge of the tree,
             * pop it off and update the corresponding weights. */
            data.pop_back();
            tree.front().pop_back();
            for (std::size_t i = 1; i < tree.size() && tree[i-1].size() > 1; ++i)
            {
                if (tree[i-1].size() % 2 == 0)
                    tree[i].pop_back();
                else
                {
                    while (i < tree.size())
                    {
                        tree[i].back() -= weight;
                        ++i;
                    }
                    return;
                }
            }
            //If we've made it here, then we need to remove a redundant head from the tree.
            tree.pop_back();
        }

        void clear(void)
        {
            for (typename std::vector<Element*>::iterator e = data.begin(); e != data.end(); ++e)
                delete *e;
            data.clear();
            tree.clear();
        }

        std::size_t size(void) const
        {
            return data.size();
        }

        bool empty(void) const
        {
            return data.empty();
        }

        void printTree(std::ostream& out = std::cout) const
        {
            if (tree.empty())
                return;
            for (std::size_t j = 0; j < tree[0].size(); ++j)
                out << "(" << data[j]->data << "," << tree[0][j] << ") ";
            out << std::endl;
            for (std::size_t i = 1; i < tree.size(); ++i)
            {
                for (std::size_t j = 0; j < tree[i].size(); ++j)
                    out << tree[i][j] << " ";
                out << std::endl;
            }
            out << std::endl;
        }

        private:

        std::vector<Element*> data;
        std::vector<std::vector<double > > tree;
    };
}

#endif

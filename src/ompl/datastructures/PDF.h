#ifndef OMPL_DATASTRUCTURES_PDF_
#define OMPL_DATASTRUCTURES_PDF_

#include <iostream>
#include <ostream>
#include <vector>

namespace ompl
{
    template <typename _T>
    class PDF
    {
        public:

        PDF(void)
        {
        }

        PDF(const std::vector<_T>& d, const std::vector<double>& weights)
        {
            //TODO throw exception if d.size() != weights.size()

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
        }

        std::size_t add(const _T& d, const double w)
        {
            //TODO throw exception if w isn't positive?
            data.push_back(d);
            if (data.size() == 1)
            {
                std::vector<double> r(1, w);
                tree.push_back(r);
                return 0;
            }
            const std::size_t index = data.size() - 1;
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
                    return index;
                }
            }
            //If we've made it here, then we need to add a new head to the tree.
            std::vector<double> head(1, tree.back()[0] + tree.back()[1]);
            tree.push_back(head);
            return index;
        }

        const _T& sample(double r) const
        {
            //TODO throw exception if tree is empty or if r is not between 0 and 1
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
            return data[node];
        }

        void remove(std::size_t index)
        {
            //TODO throw exception if index<0 or index>data.size()-1
            if (data.size() == 1)
            {
                data.clear();
                tree.clear();
                return;
            }

            std::swap(data[index], data.back());
            std::swap(tree.front()[index], tree.front().back());

            double weight;
            /* If index and back() are siblings in the tree, then
             * we don't need to make an extra pass over the tree.
             * The amount by which we change the values at the edge
             * of the tree is different in this case. */
            if (index+1 == data.size()-1 && index%2 == 0)
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
                out << "(" << data[j] << "," << tree[0][j] << ") ";
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

        std::vector<_T> data;
        std::vector<std::vector<double > > tree;
    };
}

#endif

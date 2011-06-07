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

		virtual ~PDF(void)
		{
		}

		std::size_t add(const _T d, const double w)
		{
			data.push_back(d);
			if (data.size() == 1)
			{
				std::vector<double> r(1, w);
				tree.push_back(r);
				return 0;
			}
			const std::size_t index = data.size() - 1;
			tree[0].push_back(w);
			for (std::size_t i = 1; i < tree.size(); ++i)
			{
				if (tree[i-1].size() % 2 == 1)
					tree[i].push_back(w);
				else
				{
					while (i < tree.size())
					{
						const std::size_t last = tree[i].size() - 1;
						tree[i][last] += w;
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

		_T sample(double r) const
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
		}

		protected:

		//data[i] has weight tree[0][i]
		std::vector<_T> data;
		std::vector<std::vector<double > > tree;
	};
}

#endif

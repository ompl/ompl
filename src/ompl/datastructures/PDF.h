#ifndef OMPL_DATASTRUCTURES_PDF_
#define OMPL_DATASTRUCTURES_PDF_

#include <vector>

namespace ompl
{
	template <typename _T>
	class PDF
	{
		public:

		PDF(void) : root(NULL), normalization(0.0)
		{
		}

		virtual ~PDF(void)
		{
			if (root != NULL)
				clearTree(root);
		}

		virtual void add(const _T d, const double w)
		{
			Node* n = new Node(data.size(), w);
			data.push_back(d);
			normalization += w;
			if (root == NULL)
				root = n;
			else if (root->isLeaf())
			{
				Node* parent = new Node(-1, w + root->weight);
				parent->left = root;
				parent->right = n;
				root = parent;
			}
			else
			{
				Node* sibling = root;
				Node* parent = NULL;
				while (!sibling->isLeaf())
				{
					sibling->weight += w;
					parent = sibling;
					if (sibling->left->weight <= sibling->right->weight)
						sibling = sibling->left;
					else
						sibling = sibling->right;
				}
				Node* newParent = new Node(-1, w + sibling->weight);
				newParent->left = sibling;
				newParent->right = n;
				if (parent->left == sibling)
					parent->left = newParent;
				else
					parent->right = newParent;
			}
			printTree(root);
		}

		/* The value r must be between 0 and 1. */
		virtual _T sample(double r) const
		{
			//TODO throw exception if tree is empty
			r *= normalization;
			Node* current = root;
			while (!current->isLeaf())
			{
				if (r < current->left->weight)
					current = current->left;
				else
				{					
					r -= current->left->weight;
					current = current->right;
				}
			}
			return data[current->dataIndex];
		}

		protected:

		struct Node
		{
			Node* left;
			Node* right;
			//If this is a leaf node, dataIndex contains index into data vector
			int dataIndex;			
			double weight;

			Node(const int index, const double w) : left(NULL), right(NULL), dataIndex(index), weight(w)
			{
			}

			Node(void) : Node(-1, 0.0)
			{
			}

			inline bool isLeaf()
			{
				return (left == NULL && right == NULL);		
			}
		};

		void clearTree(Node* n)
		{
			if (!n->isLeaf())
			{
				clearTree(n->left);	
				clearTree(n->right);
			}
			delete n;
		}

		void printTree(Node* n, const int level = 0)
		{
			for (int i = 0; i < level; ++i)
				std::cout << " ";
			std::cout << n->weight;
			if (n->isLeaf())
				std::cout << " " << n->dataIndex << std::endl;
			else
			{
				std::cout << std::endl;
				printTree(n->left, level+2);
				printTree(n->right, level+2);
			}
		}

		Node* root;
		double normalization;
		std::vector<_T> data;
	};
}

#endif

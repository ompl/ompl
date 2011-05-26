#ifndef DECOMPOSITION_H
#define DECOMPOSITION_H

#include <iostream>
#include <set>
#include <vector>
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/State.h"

namespace ompl {
	class Decomposition {
		public:
		/* Each decomposition region is a set of states. */
		typedef std::set<base::State*> Region;

		/* A decomposition consists of a fixed number of regions and fixed bounds. */
		Decomposition(const int numRegions, const base::RealVectorBounds &b) : regions(numRegions), bounds(b) {
		}

		virtual ~Decomposition() {
			/* Since the states in each region will come from the planner,
			 * we let the planner take care of freeing them. */
		}

		virtual int getNumRegions() const {
			return regions.size();
		}

		virtual void print() const {
			for (unsigned int i = 0; i < regions.size(); ++i) {
				std::cout << "Region " << i << " consists of " << regions[i].size() << " states." << std::endl;
			}
		}

		/* Returns the ID of the decomposition region containing the state s.
		 * Most often, this is obtained by projecting s into the workspace and finding the appropriate region. */
		virtual int locateRegion(const base::State *s) = 0;

		protected:
		std::vector<Region> regions;
		const base::RealVectorBounds &bounds;
	};
}

#endif

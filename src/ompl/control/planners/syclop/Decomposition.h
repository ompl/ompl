#ifndef DECOMPOSITION_H
#define DECOMPOSITION_H

#include <iostream>
#include <set>
#include <vector>
#define BOOST_NO_HASH
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/State.h"

namespace ompl {
	/* A decomposition is going to be Syclop-specific, for now.
	 * I could consider moving this class within the Syclop base class, but in the future,
	 * decompositions are going to be used in LTL and hybrid system approaches.
	 * Need to find a nice solution for this. */

	class Decomposition {
		public:
		/* Each decomposition region is a set of states. */
		class Region {
			public:
			std::set<base::State*> states;
			int numSelections;
			int coverage;
			std::set<int> coverageCells;
			double volume;
			double freeVolume;
			int numValid;
			int numInvalid;

			Region() : numSelections(0), coverage(0), numValid(0), numInvalid(0) {
			}
			virtual ~Region() {
			}
		};

		/* A decomposition consists of a fixed number of regions and fixed bounds. */
		Decomposition(const int numRegions, const base::RealVectorBounds &b) : regions(numRegions), bounds(b) {
		}

		virtual ~Decomposition() {
		}

		virtual int getNumRegions() const {
			return regions.size();
		}

		virtual void print() const {
			for (unsigned int i = 0; i < regions.size(); ++i) {
				std::cout << "Region " << i << " consists of " << regions[i].states.size() << " states." << std::endl;
			}
		}

		Region& operator[](const int rid) {
			return regions[rid];
		}

		/* Returns the ID of the decomposition region containing the state s.
		 * Most often, this is obtained by projecting s into the workspace and finding the appropriate region. */
		virtual int locateRegion(const base::State *s) = 0;

		/* Stores the neighboring regions of region into the vector neighbors. */
		virtual void getNeighbors(const int rid, std::vector<int>& neighbors) = 0;

		protected:
		/* Returns true iff regions r and s are physically adjacent in this decomposition. */
		virtual bool areNeighbors(int r, int s) = 0;

		std::vector<Region> regions;
		const base::RealVectorBounds &bounds;
	};
}

#endif

#ifndef GRIDDECOMPOSITION_H
#define GRIDDECOMPOSITION_H

#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/State.h"
#include "ompl/datastructures/Decomposition.h"
#include "ompl/datastructures/Grid.h"

namespace ompl {
	class GridDecomposition : public Decomposition {

		public:
		/* For now, we are assuming a 2-dimensional N-by-N grid decomposition. This will change. */
		GridDecomposition(const int len, const base::RealVectorBounds &b) : Decomposition(len*len, b), grid(2), length(len) {
			buildGrid(len);
		}

		virtual ~GridDecomposition() {
		}

		virtual void print() const {
			Decomposition::print();
			grid.status();
		}

		/* Projecting a state into whatever space on which this decomposition is defined is a problem-specific issue.
		 * Implementations of this function can call the protected overloaded version, which takes a coordinate with the same
		 * dimension as the grid. */
		virtual int locateRegion(const base::State *s) = 0;

		protected:		
		Grid<Region*> grid;

		/* Locate the region in the grid containing the point determined by coord. Since
		 * we are assuming a 2-dimensional grid, coord.size() will be 2 for now. */
		virtual int locateRegion(const std::vector<double> coord) {
			std::vector<int> gridLoc(coord.size());
			/* Once I verify that this works, I will collapse the two loops into one. */
			for (unsigned int i = 0; i < gridLoc.size(); ++i)
				gridLoc[i] = (int) ( length*(coord[i] - bounds.low[i]) / (bounds.high[i]-bounds.low[i]) );
			int region = 0;
			int factor = 1;
			for (int i = gridLoc.size()-1; i >= 0; --i) {
				region += factor*gridLoc[i];
				factor *= length;
			}
			return region;
		}

		private:
		void buildGrid(const int n) {
			Grid<Region*>::Coord coord(2);
			for (int i = 0; i < n; ++i) {
				coord[0] = i;
				for (int j = 0; j < n; ++j) {
					coord[1] = j;
					Grid<Region*>::Cell *cell = grid.createCell(coord);
					cell->data = &regions[i*n + j];
					grid.add(cell);
				} 
			}
		}

		const int length;
	};
}

#endif

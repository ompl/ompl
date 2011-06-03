#ifndef GRIDDECOMPOSITION_H
#define GRIDDECOMPOSITION_H

#include <cstdlib>
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/State.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/datastructures/Grid.h"

namespace ompl {
	class GridDecomposition : public Decomposition {

		public:
		/* In some areas of this class, we are assuming a 2-dimensional grid. This will change. */
		GridDecomposition(const int len, const int dim, const base::RealVectorBounds &b) : Decomposition(len*len, b), grid(2), length(len), dimension(dim) {
			buildGrid(len);
			cellVolume = 1.0;
			for (int i = 0; i < dim; ++i)
				cellVolume *= (b.high[i] - b.low[i]) / len;
		}

		virtual ~GridDecomposition() {
		}

		virtual void print() const {
			Decomposition::print();
			grid.status();
		}

		virtual double getRegionVolume(const int rid) const {
			return cellVolume;
		}

		/* Projecting a state into whatever space on which this decomposition is defined is a problem-specific issue.
		 * Implementations of this function can call the protected overloaded version, which takes a coordinate with the same
		 * dimension as the grid. */
		virtual int locateRegion(const base::State *s) = 0;

		/* This implementation requires time linear with the number of regions.
		 * We can do constant time if we know the dimension offline (oopsmp-syclop has cases for 2 and 3),
		 * but can we beat linear time with arbitrary dimension? */
		virtual void getNeighbors(const int rid, std::vector<int>& neighbors) {
			for (int s = 0; s < getNumRegions(); ++s) {
				if (areNeighbors(rid, s))
					neighbors.push_back(s);
			}
		}

		protected:
		/* Using datastructures/grid may not even be necessary. */
		Grid<Region*> grid;

		/* Locate the region in the grid containing the point determined by coord. Since
		 * we are assuming a 2-dimensional grid, coord.size() will be 2 for now. */
		virtual int locateRegion(const std::vector<double> coord) {
			int region = 0;
			int factor = 1;
			for (int i = coord.size()-1; i >= 0; --i) {
				const int index = (int) (length*(coord[i]-bounds.low[i])/(bounds.high[i]-bounds.low[i]));
				region += factor*index;
				factor *= length;
			}
			return region;
		}

		virtual bool areNeighbors(int r, int s) {
			if (r == s)
				return false;
			std::vector<int> rc;
			std::vector<int> sc;
			regionToCoord(r, rc);
			regionToCoord(s, sc);

			/*std::cerr << "Region " << r << " is coord (";
			for (int i = 0; i < dimension-1; ++i)
				std::cerr << rc[i] << ",";
			std::cerr << rc[dimension-1] << ")" << std::endl;

			std::cerr << "Region " << s << " is coord (";
			for (int i = 0; i < dimension-1; ++i)
				std::cerr << sc[i] << ",";
			std::cerr << sc[dimension-1] << ")" << std::endl;*/

			for (int i = 0; i < dimension; ++i) {
				if (abs(rc[i]-sc[i]) > 1)
					return false;
			}
			return true;
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

			const int dim = 2;
			double vol = 1;
			for (int d = 0; d < dim; ++d)
				vol *= (bounds.high[d]-bounds.low[d])/length;
			for (int i = 0; i < n*n; ++i)
				regions[i].volume = vol;
		}

		/* Convert a region ID to a grid coordinate, which is a vector of length equivalent to the dimension of the grid. */
		void regionToCoord(int rid, std::vector<int>& coord) {
			//TODO: Should we ensure that 0 <= rid < getNumRegions()?
			coord.resize(dimension);
			for (int i = dimension-1; i >= 0; --i) {
				int remainder = rid % length;
				coord[i] = remainder;
				rid /= length;
			}
		}

		const int length;
		const int dimension;
		double cellVolume;
	};
}

#endif

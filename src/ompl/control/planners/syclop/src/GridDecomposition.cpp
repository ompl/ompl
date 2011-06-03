#include "ompl/control/planners/syclop/GridDecomposition.h"

ompl::control::GridDecomposition::GridDecomposition(const int len, const int dim, const base::RealVectorBounds &b) :
	Decomposition(len*len, b), grid(2), length(len), dimension(dim) {
	buildGrid(len);
	cellVolume = 1.0;
	for (int i = 0; i < dim; ++i)
		cellVolume *= (b.high[i] - b.low[i]) / len;
}

ompl::control::GridDecomposition::~GridDecomposition() {
}

void ompl::control::GridDecomposition::print() const {
	Decomposition::print();
	grid.status();
}

double ompl::control::GridDecomposition::getRegionVolume(const int rid) const {
	return cellVolume;
}

/* This implementation requires time linear with the number of regions.
 * We can do constant time if we know the dimension offline (oopsmp-syclop has cases for 2 and 3),
 * but can we beat linear time with arbitrary dimension? */
void ompl::control::GridDecomposition::getNeighbors(const int rid, std::vector<int>& neighbors) {
	for (int s = 0; s < getNumRegions(); ++s) {
		if (areNeighbors(rid, s))
			neighbors.push_back(s);
	}
}

int ompl::control::GridDecomposition::locateRegion(const std::vector<double> coord) {
	int region = 0;
	int factor = 1;
	for (int i = coord.size()-1; i >= 0; --i) {
		const int index = (int) (length*(coord[i]-bounds.low[i])/(bounds.high[i]-bounds.low[i]));
		region += factor*index;
		factor *= length;
	}
	return region;
}

bool ompl::control::GridDecomposition::areNeighbors(int r, int s) {
	if (r == s)
		return false;
	std::vector<int> rc;
	std::vector<int> sc;
	regionToCoord(r, rc);
	regionToCoord(s, sc);
	for (int i = 0; i < dimension; ++i) {
		if (abs(rc[i]-sc[i]) > 1)
			return false;
	}
	return true;
}

void ompl::control::GridDecomposition::buildGrid(const int n) {
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

void ompl::control::GridDecomposition::regionToCoord(int rid, std::vector<int>& coord) {
	//TODO: Should we ensure that 0 <= rid < getNumRegions()?
	coord.resize(dimension);
	for (int i = dimension-1; i >= 0; --i) {
		int remainder = rid % length;
		coord[i] = remainder;
		rid /= length;
	}
}

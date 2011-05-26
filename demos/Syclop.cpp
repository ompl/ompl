#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/State.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/datastructures/GridDecomposition.h>

#include <iostream>

int main(void) {
	/* TODO:
	 *	Create a 4-by-4 grid decomposition of the two dimensional workspace [-1,1] x [-1,1].
	 * 	Implement a decomposition of this workspace, inheriting from GridDecomposition. */
	ompl::base::RealVectorBounds bounds(2);
	bounds.setLow(-1);
	bounds.setHigh(1);
	return 0;
}

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/datastructures/GridDecomposition.h>
#include <iostream>
#include <vector>

namespace ob = ompl::base;

class TestDecomposition : public ompl::GridDecomposition {
	public:
	TestDecomposition(const int length, ob::RealVectorBounds &bounds) : ompl::GridDecomposition(length, bounds) {
	}
	virtual ~TestDecomposition() {
	}

	virtual int locateRegion(const ob::State *s) {
		const ob::CompoundState *cs = s->as<ob::CompoundState>();
		const ob::SE2StateSpace::StateType *ws = cs->as<ob::SE2StateSpace::StateType>(0);
		std::vector<double> coord(2);
		coord[0] = ws->getX();
		coord[1] = ws->getY();
		return GridDecomposition::locateRegion(coord);
	}
};

int main(void) {
	ompl::base::RealVectorBounds bounds(2);
	bounds.setLow(-1);
	bounds.setHigh(1);
	TestDecomposition grid(4, bounds);

	ob::StateSpacePtr manifold(new ob::CompoundStateSpace());
	ob::StateSpacePtr locSpace(new ob::SE2StateSpace());
	ob::StateSpacePtr velSpace(new ob::RealVectorStateSpace(1));
	manifold->as<ob::CompoundStateSpace>()->addSubSpace(locSpace, 0.8);
	manifold->as<ob::CompoundStateSpace>()->addSubSpace(velSpace, 0.2);

	ob::ScopedState<ob::CompoundStateSpace> init(manifold);
	ob::SE2StateSpace::StateType *se = init->as<ob::SE2StateSpace::StateType>(0);
	se->setX(-0.75);
	se->setY(0.8);
	se->setYaw(0);
	init->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0;

	ob::ScopedState<ob::CompoundStateSpace> goal(init);
	se = goal->as<ob::SE2StateSpace::StateType>(0);
	se->setX(0.65);
	se->setY(-0.7);

	int initRegion = grid.locateRegion(init.get());
	std::cout << "initial state located in region " << initRegion << std::endl;
	int goalRegion = grid.locateRegion(goal.get());
	std::cout << "goal state located in region " << goalRegion << std::endl;
	
	return 0;
}

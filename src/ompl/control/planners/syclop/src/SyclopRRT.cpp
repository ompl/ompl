#include "ompl/control/planners/syclop/SyclopRRT.h"

ompl::control::SyclopRRT::SyclopRRT(const SpaceInformationPtr &si, Decomposition &d) : Syclop(si,d) {
}

ompl::control::SyclopRRT::~SyclopRRT() {
}

void ompl::control::SyclopRRT::setup(void) {
				Syclop::setup();
}

bool ompl::control::SyclopRRT::solve(const base::PlannerTerminationCondition &ptc) {
	return false;
}

void ompl::control::SyclopRRT::computeAvailableRegions(const std::vector<Region*>& lead, std::set<Region*>& avail) {
}

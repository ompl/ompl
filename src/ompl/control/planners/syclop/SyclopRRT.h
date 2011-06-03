#ifndef SYCLOPRRT_H
#define SYCLOPRRT_H

#include "ompl/control/planners/syclop/Syclop.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"

namespace ompl {
	namespace control {
		class SyclopRRT : public Syclop {
			public:
			SyclopRRT(const SpaceInformationPtr &si, Decomposition &d) : Syclop(si,d) {
			}

			virtual ~SyclopRRT() {
			}

			virtual void setup(void) {
				Syclop::setup();
			}

			virtual bool solve(const base::PlannerTerminationCondition &ptc) {
				return false;
			}

			virtual void computeAvailableRegions(const std::vector<Region*>& lead, std::set<Region*>& avail) {

			}
		};
	}
}

#endif

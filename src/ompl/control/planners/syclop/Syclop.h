#ifndef SYCLOP_H
#define SYCLOP_H

#include "ompl/control/planners/PlannerIncludes.h"

namespace ompl {
	namespace control {
		class Syclop : public base::Planner {
			public:
			Syclop(const SpaceInformationPtr &si) : base::Planner(si, "Syclop") {
			}

			virtual ~Syclop() {
			}
		};
	}
}

#endif

#ifndef SYCLOP_H
#define SYCLOP_H

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/control/planners/syclop/Decomposition.h"

namespace ompl {
	namespace control {
		class Syclop : public base::Planner {
			public:
			typedef Decomposition::Region Region;

			Syclop(const SpaceInformationPtr &si, Decomposition &d) : base::Planner(si, "Syclop"), decomp(d) {
			}

			virtual ~Syclop() {
			}

			virtual void computeLead(std::vector<Region*>& lead) {
				
			}

			virtual Region* selectRegion(const std::set<Region*>& regions);
			virtual void computeAvailableRegions(const std::vector<Region*>& lead, std::set<Region*>& avail) = 0;
			

			protected:
			Decomposition &decomp;
		};
	}
}

#endif

#ifndef SYCLOPRRT_H
#define SYCLOPRRT_H

#include "ompl/control/planners/syclop/Syclop.h"
#include "ompl/datastructures/GridDecomposition.h"

namespace ompl {
	namespace control {
		class SyclopRRT : public Syclop {
			public:
			SyclopRRT(const SpaceInformationPtr &si) : Syclop(si) {
			}

			virtual ~SyclopRRT() {
			}
		};
	}
}

#endif

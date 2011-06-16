#ifndef SYCLOPRRT_H
#define SYCLOPRRT_H

#include "ompl/control/planners/syclop/Syclop.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"

namespace ompl
{
    namespace control
    {
        class SyclopRRT : public Syclop
        {
            public:
            SyclopRRT(const SpaceInformationPtr &si, Decomposition &d);
            virtual ~SyclopRRT();
            virtual void setup(void);

            protected:
            virtual void initializeTree(const base::State *s);
            virtual void selectAndExtend(int region, std::set<Motion*> newMotions);
        };
    }
}
#endif

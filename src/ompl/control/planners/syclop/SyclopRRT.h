#ifndef SYCLOPRRT_H
#define SYCLOPRRT_H

#include "ompl/control/planners/syclop/Syclop.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"

namespace ompl
{
    namespace control
    {
        /* TODO could syclopRRT inherit from both Syclop and RRT, to use some of RRT's helper and setup methods?
            I imagine SyclopRRT would override solve() to call Syclop::solve(). It would provide initializeTree() and selectAndExtend(),
            which are called by Syclop::solve(). ...for now, just rewrite some RRT code. */
        class SyclopRRT : public Syclop
        {
            public:
            SyclopRRT(const SpaceInformationPtr &si, Decomposition &d);
            virtual ~SyclopRRT(void);
            virtual void setup(void);
            virtual void clear(void);

            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            double getGoalBias(void) const
            {
                return goalBias_;
            }

            virtual void getPlannerData(base::PlannerData &data) const;

            protected:
            virtual Syclop::Motion* initializeTree(const base::State *s);
            virtual void selectAndExtend(Region& region, std::set<Motion*>& newMotions);
            void freeMemory(void);

            base::StateSamplerPtr sampler_;
            ControlSamplerPtr controlSampler_;
            std::vector<Motion*> motions;
        };
    }
}
#endif

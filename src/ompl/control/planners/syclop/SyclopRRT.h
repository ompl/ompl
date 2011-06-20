#ifndef SYCLOPRRT_H
#define SYCLOPRRT_H

#include "ompl/control/planners/syclop/Syclop.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"
#include "ompl/datastructures/NearestNeighbors.h"

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

            template<template<typename T> class NN>
            void setNearestNeighbors(void)
            {
                nn_.reset(new NN<Motion*>());
            }

            protected:
            virtual Syclop::Motion* initializeTree(const base::State *s);
            virtual void selectAndExtend(Region& region, std::set<Motion*>& newMotions);
            void freeMemory(void);

            double distanceFunction(const Motion* a, const Motion* b) const
            {
                return si_->distance(a->state, b->state);
            }

            base::StateSamplerPtr sampler_;
            ControlSamplerPtr controlSampler_;
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;
            double goalBias_;
        };
    }
}
#endif

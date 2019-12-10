#ifndef OMPL_GEOMETRIC_PLANNERS_FIBOP_
#define OMPL_GEOMETRIC_PLANNERS_FIBOP_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/Path.h>
#include <type_traits>

namespace ompl
{
    namespace geometric
    {
        //FiberOP: Fiber-bundle optimizer
        class FiberOP : public base::Planner
        {
            using BaseT = ompl::base::Planner;
            using FiberBundle = std::vector<ompl::base::SpaceInformationPtr>;

        public:

            FiberOP( ompl::base::SpaceInformationPtr si, std::vector<FiberBundle> &fiberBundles, std::string type = "FiberBundleOptimizer");

            ~FiberOP() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            void setup() override;

        protected:
            std::vector<base::PlannerPtr> unfilteredPlanners_;
            std::vector<base::PlannerPtr> filteredPlanners_;
            std::vector<FiberBundle> fiberBundles_;
            unsigned int iteration_;

            FiberBundle fiberBundleGlobalOptimum_;

        };
    }
}

#endif



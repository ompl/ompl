#ifndef OMPL_GEOMETRIC_PLANNERS_OPTIMIZER_
#define OMPL_GEOMETRIC_PLANNERS_OPTIMIZER_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/Path.h>
namespace ompl
{
    namespace control
    {
        class Optimizer : public base::Planner
        {
        public:
            Optimizer(const base::SpaceInformationPtr &si, const base::PathPtr &path);

            ~Optimizer() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            void setup() override;
        protected:
            bool IsGeometric() const;
            base::PathPtr path_;

        };
    }
}

#endif


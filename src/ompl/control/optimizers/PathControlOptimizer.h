#ifndef OMPL_GEOMETRIC_PLANNERS_OPTIMIZER_PATHCONTROL__
#define OMPL_GEOMETRIC_PLANNERS_OPTIMIZER_PATHCONTROL__

#include <ompl/control/PathControl.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>

namespace ompl
{
    namespace control
    {
        class PathControlOptimizer
        {
          public:

            PathControlOptimizer(control::SpaceInformationPtr si, const base::OptimizationObjectivePtr& obj=nullptr);
            void simplify(PathControl* path);

          protected:

            control::SpaceInformationPtr si_;

            base::OptimizationObjectivePtr obj_;
        };
    }
}

#endif

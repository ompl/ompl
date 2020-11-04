#ifndef OMPL_MULTILEVEL_PLANNERS_BundleSpace_PathSpaceQMP_
#define OMPL_MULTILEVEL_PLANNERS_BundleSpace_PathSpaceQMP_
#include <ompl/multilevel/planners/qmp/QMPStarImpl.h>
#include <ompl/multilevel/planners/explorer/datastructures/PathSpace.h>
#include <ompl/datastructures/PDF.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(PathVisibilityChecker);

        class PathSpaceQMP : public ompl::multilevel::PathSpace, public ompl::multilevel::QMPStarImpl
        {
            using BaseT = QMPStarImpl;

        public:
            PathSpaceQMP(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);
            virtual ~PathSpaceQMP() override;
            virtual void grow() override;

        protected:
            double bestCost_{std::numeric_limits<double>::infinity()};

            PathVisibilityChecker *pathVisibilityChecker_{nullptr};
        };
    }  // namespace multilevel
}  // namespace ompl

#endif

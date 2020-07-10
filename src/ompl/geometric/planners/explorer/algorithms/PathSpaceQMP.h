#ifndef OMPL_GEOMETRIC_PLANNERS_BundleSpace_PathSpaceQMP_
#define OMPL_GEOMETRIC_PLANNERS_BundleSpace_PathSpaceQMP_
#include <ompl/geometric/planners/multilevel/algorithms/QMPStarImpl.h>
#include <ompl/geometric/planners/explorer/datastructures/PathSpace.h>
#include <ompl/datastructures/PDF.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathVisibilityChecker);

        class PathSpaceQMP : public ompl::geometric::PathSpace, 
                             public ompl::geometric::QMPStarImpl
        {
            using BaseT = QMPStarImpl;

        public:
            PathSpaceQMP(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);
            virtual ~PathSpaceQMP() override;
            virtual void grow() override;

            PathVisibilityChecker *getPathVisibilityChecker();
            PathVisibilityChecker *pathVisibilityChecker_{nullptr};

        protected:
            double bestCost_{std::numeric_limits<double>::infinity()};


        };
    }  // namespace geometric
}  // namespace ompl

#endif

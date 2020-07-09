#ifndef OMPL_GEOMETRIC_PLANNERS_BundleSpace_MotionExplorerQMPImpl_
#define OMPL_GEOMETRIC_PLANNERS_BundleSpace_MotionExplorerQMPImpl_
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

        class MotionExplorerQMPImpl : public ompl::geometric::PathSpace, 
                                      public ompl::geometric::QMPStarImpl
        {
            using BaseT = QMPStarImpl;

        public:
            MotionExplorerQMPImpl(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);
            virtual ~MotionExplorerQMPImpl() override;
            virtual void grow() override;
            // virtual void firstGrow();
            // virtual void getPlannerData(ompl::base::PlannerData &data) const override;

            PathVisibilityChecker *getPathVisibilityChecker();
            PathVisibilityChecker *pathVisibilityChecker_{nullptr};

            // virtual unsigned int getNumberOfPaths() const override;

        protected:
            double bestLength_{std::numeric_limits<double>::infinity()};


        };
    }  // namespace geometric
}  // namespace ompl

#endif

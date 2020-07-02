#ifndef OMPL_GEOMETRIC_PLANNERS_BundleSpace_EXPLORERTESTIMPL_
#define OMPL_GEOMETRIC_PLANNERS_BundleSpace_EXPLORERTESTIMPL_
#include <ompl/geometric/planners/multilevel/algorithms/QMPStarImpl.h>
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

        class ExplorerTestImpl : public ompl::geometric::QMPStarImpl
        {
            using BaseT = QMPStarImpl;

        public:
            ExplorerTestImpl(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);
            virtual ~ExplorerTestImpl() override;
            virtual void grow() override;
            virtual void firstGrow();

            PathVisibilityChecker* getPathVisibilityChecker();
            PathVisibilityChecker* pathVisibilityChecker_{nullptr};




        protected:
            std::vector<std::vector<Vertex>> solutionspaths;
            std::vector<float> solutionPathLength;

        };
    }  // namespace geometric
}  // namespace ompl

#endif

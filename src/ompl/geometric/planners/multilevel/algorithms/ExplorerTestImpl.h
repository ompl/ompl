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
            virtual void getPlannerData(ompl::base::PlannerData &data) const override;

            PathVisibilityChecker *getPathVisibilityChecker();
            PathVisibilityChecker *pathVisibilityChecker_{nullptr};

            void setSelectedPath(int);
            int getSelectedPath();
            unsigned int getNumberOfPaths() const;



        protected:
            std::vector<std::vector<Vertex>> solutionspaths;
            std::vector<float> solutionPathLength;
            std::vector<std::vector<ompl::base::State *>> pathStackHead_;
            int selectedPath_{-1}; //selected path to sample from (if children try to sample this space)


        };
    }  // namespace geometric
}  // namespace ompl

#endif

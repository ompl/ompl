#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_STARIMPL_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_STARIMPL_
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/multilevel/planners/qrrt/BiQRRTImpl.h>
#include <ompl/multilevel/planners/qrrt/SparseTree.h>
#include <ompl/datastructures/PDF.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace multilevel
    {
        class STARImpl : public ompl::multilevel::BundleSpaceGraph
        {
            using BaseT = BundleSpaceGraph;
            using TreeData = std::shared_ptr<NearestNeighbors<Configuration *>>;

        public:
            STARImpl(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);
            ~STARImpl() override;

            void grow() override;
            void init() override;
            void setup() override;
            void clear() override;
            void getPlannerData(ompl::base::PlannerData &data) const override;

            void addToTree(SparseTreePtr& tree, Configuration *x);
            // void updateUnsuccessfulExtension(SparseTreePtr& tree, Configuration *x);
            // void updateSuccessfulExtension(SparseTreePtr& tree, Configuration *x);
            // void updateExtension(SparseTreePtr& tree, Configuration *x);

            bool isInfeasible() override;
            bool hasConverged() override;
            double getImportance() const override;
        protected:
            // std::vector<int> treeElement_numberOfExtensions_;
            // std::vector<int> treeElement_numberOfSuccessfulExtensions_;
            // std::vector<int> treeElement_numberOfUnsuccessfulSubsequentExtensions_;
            // std::vector<bool> treeElement_isConverged_;

            SparseTreePtr treeStart_;
            SparseTreePtr treeGoal_;
            double distanceBetweenTrees_;
            bool activeInitialTree_{true};
        };
    }  // namespace multilevel
}  // namespace ompl

#endif

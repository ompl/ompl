#pragma once

#include <ompl/multilevel/datastructures/BundleSpaceGraphSparse.h>
#include <ompl/multilevel/planners/explorer/datastructures/PathSpace.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/control/Control.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/DirectedControlSampler.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace multilevel
    {

        OMPL_CLASS_FORWARD(PathVisibilityChecker);

        class PathSpaceSparseOptimization : 
            public multilevel::PathSpace, 
            public multilevel::BundleSpaceGraphSparse
        {
            using BaseT = ompl::multilevel::BundleSpaceGraphSparse;

        public:
            PathSpaceSparseOptimization(const base::SpaceInformationPtr &si, BundleSpace *parent_);
            virtual ~PathSpaceSparseOptimization() override;
            virtual void grow() override;
            virtual void growGeometric();
            virtual void growControl();
            virtual bool getSolution(base::PathPtr &solution) override;

            virtual void setup() override;
            virtual void clear() override;

            void setGoalBias(double goalBias);
            double getGoalBias() const;
            void setRange(double distance);
            double getRange() const;
            virtual bool hasSolution() override;

            Configuration *q_random{nullptr};
            ompl::control::Control *c_random{nullptr};
            base::State *s_random;
            ompl::control::StatePropagatorPtr prop;
            ompl::control::DirectedControlSamplerPtr dCSampler;

            virtual void getPlannerData(base::PlannerData &data) const override;
            unsigned int getNumberOfPaths() const;

            //############################################################################
            void printAllPathsUtil(Vertex u, Vertex d, bool visited[], int path[], int &path_index);
            virtual void enumerateAllPaths();
            void removeReducibleLoops();
            void removeEdgeIfReductionLoop(const Edge &e);
            const std::vector<base::State *> getKthPath(uint k) const;
            void getPathIndices(const std::vector<base::State *> &states, std::vector<int> &idxPath) const;
            bool isProjectable(const std::vector<base::State *> &pathBundle) const;
            int getProjectionIndex(const std::vector<base::State *> &pathBundle) const;
            virtual void sampleFromDatastructure(base::State *q_random_graph) override;

            void pushPathToStack(std::vector<base::State *> &path);
            // void removeLastPathFromStack();
            std::vector<base::State *> getProjectedPath(const std::vector<base::State *> pathBundle,
                                                        const base::SpaceInformationPtr &si) const;

            void freePath(std::vector<base::State *> path, const base::SpaceInformationPtr &si) const;

            unsigned numberOfFailedAddingPathCalls{0};
            unsigned Nhead{5};  // head -nX (to display only X top paths)
            std::vector<geometric::PathGeometric> pathStack_;
            std::vector<std::vector<base::State *>> pathStackHead_;
            void PrintPathStack();

            //############################################################################
            PathVisibilityChecker *pathVisibilityChecker_{nullptr};

        protected:
            double pathBias_{0.};
            double pathBiasFraction_{0.05};
            int numberOfControlSamples{10};
            double propStepSize;
            int controlDuration{10};
            double maxDistance{.0};
            double goalBias{.05};
            // double epsilon{.0};
            double distanceToGoal{.0};
            double approximateDistanceToGoal{.0};

            base::OptimizationObjectivePtr pathObj_{nullptr};
        };
    };
};

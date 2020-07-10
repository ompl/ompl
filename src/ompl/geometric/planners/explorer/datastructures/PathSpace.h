#pragma once
#include <ompl/geometric/planners/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/base/PlannerData.h>

namespace ompl{

    namespace geometric{

        class PathVisibilityChecker;
        class PathSpace{

            using VertexPath = std::vector<BundleSpaceGraph::Vertex>;

          public:

            PathSpace() = delete;
            PathSpace(BundleSpaceGraph*);
            ~PathSpace();

            void setSelectedPath(int);

            int getSelectedPath();

            unsigned int getNumberOfPaths() const;

            void updatePath(int k, VertexPath p, double cost);

            void addPath(VertexPath p, double cost);

            double getPathCost(int k) const;

            std::vector<BundleSpaceGraph::Vertex>& getCriticalPath(int k);

            void getPlannerData(base::PlannerData &data, BundleSpaceGraph* bundleGraph) const;

            PathVisibilityChecker *getPathVisibilityChecker();

          protected:
            BundleSpaceGraph *bundleSpaceGraph_;

            PathVisibilityChecker *pathVisibilityChecker_{nullptr};

            int selectedPath_{-1};

            std::vector<VertexPath> criticalPaths_;

            std::vector<double> criticalPathsCost_;
        };
    }
}
